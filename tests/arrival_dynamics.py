"""Deterministic closed-loop stress bench, NOT the Webots physics engine.

Uses the production goal controller with delayed measurements/commands,
first-order motor response, a small deadband and seeded pose noise.
"""
from collections import deque
import importlib.util
import inspect
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'ros_ws/src/fabtino_navigation'))
from fabtino_navigation import goal_control


def simulate(module, angle, lag, delay, noise, distance=1., heading=0., seed=1):
    rng = random.Random(seed)
    dt = .02
    follower = module.GoalFollower(goal_tolerance_m=.05)
    goal = module.Goal.from_request({'x': distance, 'y': 0, 'yaw_deg': angle})
    follower.set_goal(goal)
    pose = [0., 0., heading]
    velocity = [0., 0.]
    count = round(delay/dt)
    commands = deque([(0., 0.)]*(count+1), maxlen=count+1)
    samples = deque([(pose.copy(), velocity.copy())]*(count+1), maxlen=count+1)
    supports_velocity = 'velocity' in inspect.signature(follower.command).parameters
    reached_at = None
    reached_velocity = None
    peak_command_after_success = 0.
    states = {}
    for step in range(round(120/dt)):
        now = step*dt
        measured, measured_velocity = samples[0]
        observation = (measured[0]+rng.uniform(-noise, noise),
                       measured[1]+rng.uniform(-noise, noise),
                       measured[2]+rng.uniform(-.003, .003))
        if step % 5 == 0:
            follower.set_path(goal.goal_id, [(measured[0], measured[1]), (distance, 0.)])
        if supports_velocity:
            requested = follower.command(observation, now, velocity=measured_velocity)
        else:
            requested = follower.command(observation, now)
        states[follower.state] = states.get(follower.state, 0)+1
        if follower.state == 'goal_reached' and reached_at is None:
            reached_at, reached_velocity = now, velocity.copy()
        if reached_at is not None:
            peak_command_after_success = max(peak_command_after_success, abs(requested[0]), abs(requested[1]))
        commands.append(requested)
        actual_command = commands[0]
        for i, deadband in enumerate((.005, .01)):
            target = actual_command[i] if abs(actual_command[i]) >= deadband else 0.
            velocity[i] += (target-velocity[i])*min(1., dt/lag)
        pose[0] += velocity[0]*math.cos(pose[2])*dt
        pose[1] += velocity[0]*math.sin(pose[2])*dt
        pose[2] = module.wrap_angle(pose[2]+velocity[1]*dt)
        samples.append((pose.copy(), velocity.copy()))
        if reached_at is not None and now-reached_at >= 3.:
            break
        if follower.state == 'goal_failed':
            break
    position_error = math.hypot(pose[0]-distance, pose[1])
    angle_error = 0. if angle is None else abs(math.degrees(module.wrap_angle(math.radians(angle)-pose[2])))
    passed = (follower.state == 'goal_reached' and position_error <= .05
              and angle_error <= 3. and abs(velocity[0]) <= .01 and abs(velocity[1]) <= .03
              and abs(reached_velocity[0]) <= .01 and abs(reached_velocity[1]) <= .03
              and peak_command_after_success == 0.)
    return dict(angle=angle, lag_s=lag, delay_s=delay, noise_m=noise, distance_m=distance,
                initial_heading=heading, passed=passed, state=follower.state, reason=follower.reason,
                position_error_m=position_error, angle_error_deg=angle_error,
                velocity_at_success=reached_velocity, final_velocity=velocity,
                reached_at_s=reached_at, states=states)


def run(module):
    results = []
    for distance in (1., 9.):
        for angle in (None, 0, 80, 90, 180, -90):
            for lag in (.10, .30, .60):
                for delay in (0., .10, .20):
                    for noise in (0., .005):
                        results.append(simulate(module, angle, lag, delay, noise, distance))
    # Include both sides of the +/-pi discontinuity across all tested delays,
    # so departure-turn stability is exercised as well as terminal stopping.
    for heading in (-math.pi, -math.pi+.002, -math.pi/2, math.pi/2, math.pi-.002, math.pi):
        for lag in (.10, .30, .60):
            for delay in (0., .10, .20):
                for noise in (0., .005):
                    results.append(simulate(module, 90, lag, delay, noise, 1., heading))
    return dict(model='synthetic unicycle, not Webots or ROS DDS', cases=len(results),
                passed=sum(r['passed'] for r in results),
                failed=sum(not r['passed'] for r in results),
                max_position_error_m=max(r['position_error_m'] for r in results if r['state']=='goal_reached'),
                max_angle_error_deg=max(r['angle_error_deg'] for r in results if r['state']=='goal_reached'),
                results=results)


if __name__ == '__main__':
    module = goal_control
    if len(sys.argv)>2:
        spec = importlib.util.spec_from_file_location('_baseline_goal', sys.argv[2])
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
    report = run(module)
    Path(sys.argv[1]).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}, indent=2))
    raise SystemExit(1 if report['failed'] else 0)
