"""ROS-independent goal validation and terminal pose control."""
from dataclasses import dataclass
import math
import uuid


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw_rad: float | None
    goal_id: str

    @classmethod
    def from_request(cls, request):
        x, y = float(request['x']), float(request['y'])
        degrees = request.get('yaw_deg')
        yaw = None if degrees is None else math.radians(float(degrees))
        if not all(math.isfinite(v) for v in (x, y)) or (yaw is not None and not math.isfinite(yaw)):
            raise ValueError('Goal coordinates and orientation must be finite')
        return cls(x, y, None if yaw is None else wrap_angle(yaw), uuid.uuid4().hex)

    def payload(self):
        return {'type': 'goal', 'x': self.x, 'y': self.y,
                'yaw_deg': None if self.yaw_rad is None else math.degrees(self.yaw_rad),
                'goal_id': self.goal_id}

    @classmethod
    def from_payload(cls, payload):
        goal = cls.from_request(payload)
        return cls(goal.x, goal.y, goal.yaw_rad, str(payload['goal_id']))


class GoalFollower:
    def __init__(self, lookahead_m=0.45, max_linear_mps=0.25,
                 max_angular_rps=1.0, heading_slowdown_rad=1.0,
                 goal_tolerance_m=0.05, yaw_tolerance_rad=math.radians(3),
                 final_max_angular_rps=0.35, settle_time_s=0.4,
                 yaw_hysteresis_rad=math.radians(2),
                 alignment_no_progress_s=5.0, alignment_timeout_s=30.0,
                 final_approach_distance_m=0.4, final_linear_mps=0.08,
                 final_heading_kp=1.2, final_heading_kd=0.35,
                 stopped_linear_mps=0.01, stopped_angular_rps=0.03,
                 path_max_angular_rps=0.7, path_heading_kd=0.35,
                 path_turn_timeout_s=20.0, path_turn_no_progress_s=5.0,
                 movement_no_progress_s=15.0):
        self.lookahead = lookahead_m
        self.max_linear = max_linear_mps
        self.max_angular = max_angular_rps
        self.heading_slowdown = heading_slowdown_rad
        self.position_tolerance = goal_tolerance_m
        self.yaw_tolerance = yaw_tolerance_rad
        self.final_max_angular = final_max_angular_rps
        self.settle_time = settle_time_s
        self.yaw_hysteresis = yaw_hysteresis_rad
        self.alignment_no_progress = alignment_no_progress_s
        self.alignment_timeout = alignment_timeout_s
        self.final_approach_distance = final_approach_distance_m
        self.final_linear = final_linear_mps
        self.final_heading_kp = final_heading_kp
        self.final_heading_kd = final_heading_kd
        self.stopped_linear = stopped_linear_mps
        self.stopped_angular = stopped_angular_rps
        self.path_max_angular = path_max_angular_rps
        self.path_heading_kd = path_heading_kd
        self.path_turn_timeout = path_turn_timeout_s
        self.path_turn_no_progress = path_turn_no_progress_s
        self.movement_no_progress = movement_no_progress_s
        self.goal = None
        self.path = []
        self.state = 'idle'
        self.position_reached = False
        self.settled_since = None
        self.reset_alignment()

    def reset_path_turn(self):
        self.turn_direction = None
        self.turn_started = None
        self.turn_last_progress = None
        self.turn_best_error = math.inf

    def reset_alignment(self):
        self.reset_path_turn()
        self.progress_pose = None
        self.progress_time = None
        self.alignment_started = None
        self.last_alignment_progress = None
        self.best_yaw_error = math.inf
        self.heading_reached = False
        self.final_turn_direction = None
        self.reason = ''

    def set_goal(self, goal):
        self.goal = goal
        self.path = []
        self.state = 'waiting_for_path'
        self.position_reached = False
        self.settled_since = None
        self.reset_alignment()

    def stop(self):
        self.goal = None
        self.path = []
        self.state = 'stopped'
        self.position_reached = False
        self.settled_since = None
        self.reset_alignment()

    def fail(self, reason):
        self.state = 'goal_failed'
        self.reason = reason
        self.path = []
        return (0.0, 0.0)

    def set_path(self, goal_id, points):
        # Delayed replans must never restart a cancelled/completed goal.
        if self.goal is None or goal_id != self.goal.goal_id or self.state in ('goal_reached', 'goal_failed'):
            return
        self.path = list(points)

    def arc_target(self, pose):
        # Project onto the closest route segment; do not steer back to its start.
        points = self.path
        if len(points) == 1:
            return points[0]
        best = None
        for index, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length2 = dx * dx + dy * dy
            t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((pose[0]-a[0])*dx + (pose[1]-a[1])*dy)/length2))
            projection = (a[0] + t*dx, a[1] + t*dy)
            distance = math.hypot(pose[0]-projection[0], pose[1]-projection[1])
            if best is None or distance < best[0]:
                best = (distance, index, projection)
        _, index, previous = best
        remaining = self.lookahead
        for point in points[index+1:]:
            length = math.hypot(point[0]-previous[0], point[1]-previous[1])
            if length >= remaining and length > 0:
                fraction = remaining / length
                return (previous[0]+fraction*(point[0]-previous[0]), previous[1]+fraction*(point[1]-previous[1]))
            remaining -= length
            previous = point
        return points[-1]

    def command(self, pose, now, velocity=None):
        if self.goal is None or self.state in ('goal_reached', 'goal_failed'):
            return (0.0, 0.0)
        if pose is None or not self.path:
            self.state = 'waiting_for_path'
            self.settled_since = None
            return (0.0, 0.0)
        x, y, yaw = pose
        measured_v, measured_w = (0.0, 0.0) if velocity is None else velocity
        distance = math.hypot(self.goal.x-x, self.goal.y-y)
        # Aim inside the requested tolerance to leave room for braking/noise.
        # The outer threshold remains the actual requested accuracy, not 2x it.
        if distance <= 0.5*self.position_tolerance+1e-9:
            self.position_reached = True
        elif distance > self.position_tolerance:
            self.position_reached = False
        if self.position_reached:
            self.reset_path_turn()
            self.progress_pose = None
            if abs(measured_v) > self.stopped_linear:
                self.state = 'braking'
                self.settled_since = None
                return (0.0, 0.0)
            error = 0.0 if self.goal.yaw_rad is None else wrap_angle(self.goal.yaw_rad-yaw)
            if abs(error) > math.radians(150):
                if self.final_turn_direction is None:
                    self.final_turn_direction = 1 if error >= 0 else -1
                if error*self.final_turn_direction < 0:
                    error += self.final_turn_direction*2*math.pi
            elif abs(error) < math.pi/2:
                self.final_turn_direction = None
            capture_yaw = self.yaw_tolerance-min(self.yaw_hysteresis, self.yaw_tolerance*.5)
            if abs(error) <= capture_yaw:
                self.heading_reached = True
            elif abs(error) > self.yaw_tolerance:
                self.heading_reached = False
            if not self.heading_reached:
                self.state = 'aligning'
                self.settled_since = None
                if self.alignment_started is None:
                    self.alignment_started = now
                    self.last_alignment_progress = now
                if abs(error) < self.best_yaw_error-math.radians(1):
                    self.best_yaw_error = abs(error)
                    self.last_alignment_progress = now
                if now-self.alignment_started >= self.alignment_timeout:
                    return self.fail('final_heading_timeout')
                if now-self.last_alignment_progress >= self.alignment_no_progress:
                    return self.fail('final_heading_no_progress')
                limit = min(self.final_max_angular, self.max_angular)
                angular = self.final_heading_kp*error-self.final_heading_kd*measured_w
                return (0.0, max(-limit, min(limit, angular)))
            # A zero command is not evidence of a stopped robot. Confirm measured
            # motion has stopped before starting the arrival confirmation timer.
            if abs(measured_w) > self.stopped_angular:
                self.state = 'braking'
                self.settled_since = None
                return (0.0, 0.0)
            if self.settled_since is None:
                self.settled_since = now
            self.state = 'settling'
            if now-self.settled_since+1e-9 >= self.settle_time:
                self.state = 'goal_reached'
                self.path = []
            return (0.0, 0.0)
        self.settled_since = None
        self.state = 'following'
        target = self.arc_target(pose)
        error = wrap_angle(math.atan2(target[1]-y, target[0]-x)-yaw)
        # Near +/-pi the shortest-turn sign flips with millimetres of pose or
        # route noise. Commit to one side until the target is clearly ahead.
        if abs(error) > math.radians(150):
            if self.turn_direction is None:
                self.turn_direction = 1 if error >= 0 else -1
            if error*self.turn_direction < 0:
                error += self.turn_direction*2*math.pi
        elif abs(error) < math.pi/2:
            self.turn_direction = None
        target_distance = math.hypot(target[0]-x, target[1]-y)
        # Slow the final unobstructed route segment, without cutting a detour
        # merely because its endpoint happens to be geometrically nearby.
        final_segment = (distance <= self.final_approach_distance and
                         math.hypot(target[0]-self.goal.x, target[1]-self.goal.y) < 1e-6)
        limit_v = min(self.max_linear, self.final_linear) if final_segment else self.max_linear
        limit_w = min(self.max_angular, self.final_max_angular if final_segment else self.path_max_angular)
        if final_segment:
            self.state = 'approaching'
        linear = min(limit_v, 0.8*target_distance, 0.8*distance)*max(0.0, math.cos(error))
        if abs(error) > self.heading_slowdown:
            linear *= max(0.0, 1.0-(abs(error)-self.heading_slowdown)/max(1e-6, math.pi-self.heading_slowdown))
        # A large initial turn gets a dedicated bounded state. Small corrections
        # still happen while advancing; final orientation is only used at goal.
        if abs(error) > 1.2 or (self.turn_started is not None and abs(error) > .45):
            self.progress_pose = None
            self.state = 'turning_to_path'
            linear = 0.0
            if self.turn_started is None:
                self.turn_started = self.turn_last_progress = now
                self.turn_best_error = abs(error)
            if abs(error) < self.turn_best_error-math.radians(2):
                self.turn_best_error = abs(error)
                self.turn_last_progress = now
            if now-self.turn_started >= self.path_turn_timeout:
                return self.fail('path_heading_timeout')
            if now-self.turn_last_progress >= self.path_turn_no_progress:
                return self.fail('path_heading_no_progress')
        else:
            self.reset_path_turn()
            # Progress means displacement along any detour, not necessarily
            # decreasing distance to the goal. Stops prolonged motor pushing.
            if self.progress_pose is None or math.hypot(x-self.progress_pose[0], y-self.progress_pose[1]) >= .03:
                self.progress_pose = (x, y)
                self.progress_time = now
            elif now-self.progress_time >= self.movement_no_progress:
                return self.fail('translation_no_progress')
        angular = 2.0*error-(self.final_heading_kd if final_segment else self.path_heading_kd)*measured_w
        return (linear, max(-limit_w, min(limit_w, angular)))
