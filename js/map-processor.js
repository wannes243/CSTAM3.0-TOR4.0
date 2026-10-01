import { MapModel } from "./map-model.js";
import { RobotState } from "./robot-state.js";

/*
 * The supplied controller sends new_points in MAP/WORLD coordinates. The
 * controller performs the explicit chain:
 *
 *   robot_x = range * cos(sensor_angle)
 *   robot_y = -range * sin(sensor_angle)
 *   world_x = pose.x + c * robot_x - s * robot_y
 *   world_y = pose.y + s * robot_x + c * robot_y
 *
 * Therefore this module deliberately does NOT apply robot yaw/translation to
 * the point coordinates a second time. It is retained only for legacy live
 * ray-tracing callers; the active app sends map points directly to renderers.
 */
export const MapProcessor = (() => {
  function process(worldPoints) {
    if (!Array.isArray(worldPoints)) return 0;

    const robot = RobotState.get();
    let valid = 0;

    for (const point of worldPoints) {
      if (!Array.isArray(point) || point.length < 2) continue;

      const x = Number(point[0]);
      const y = Number(point[1]);

      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;

      MapModel.rayTrace(robot.x, robot.y, x, y);
      valid++;
    }

    return valid;
  }

  return { process };
})();
