export const RobotState = (() => {
  let state = {
    x: 0,
    y: 0,
    yaw: 0,
    lidarZ: 0.285
  };

  function finiteOr(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function update(data = {}) {
    state.x = finiteOr(data.x, state.x);
    state.y = finiteOr(data.y, state.y);
    state.yaw = finiteOr(data.yaw ?? data.theta, state.yaw);
    state.lidarZ = finiteOr(data.lidar_z, state.lidarZ);
  }

  function resetXY() {
    state.x = 0;
    state.y = 0;
  }

  function reset() {
    state.x = 0;
    state.y = 0;
    state.yaw = 0;
  }

  function get() {
    return { ...state };
  }

  return { update, resetXY, reset, get };
})();
