import { WebSocketManager } from "./websocket-manager.js?v=pipeline-v7";
import { RobotState } from "./robot-state.js?v=pipeline-v7";
import { MapModel } from "./map-model.js?v=delivery-v8";
import { GridRenderer } from "./grid-renderer.js?v=delivery-v8";
import { ThreeDRenderer } from "./three-renderer.js?v=delivery-v8";
import { MapExporter, readPNGMetadata, validateBundle, validateNavigation } from "./map-exporter.js?v=delivery-v8";
import { createNavigationFeatures } from "./navigation-features.js?v=delivery-v8";
import { UIController } from "./ui-controller.js?v=delivery-v8";

const $ = UIController.$;
let MAX_LINEAR = 0.50;
let MAX_ANGULAR = 1.50;

let scanning = false;
let packetCount = 0;
let totalHits = 0;
let knownMapLoaded = false;
let pendingKnownMap = null;
let lastNavigationDebug = "";
let viewerConfigApplied = false;
let pendingGoal = null;
let requestedView = "2d";
let pendingOperation = null;
let operationTimer = null;
let activeMapId = null;
let features = null;
const drivePointers = new Map();
const driveKeys = new Map();

const held = {
  up: 0,
  down: 0,
  left: 0,
  right: 0
};

function sendDrive() {
  const v = (held.up - held.down) * MAX_LINEAR;
  const w = (held.left - held.right) * MAX_ANGULAR;
  if (pendingOperation && (v || w)) {
    UIController.updateDrive(held, false, "Wait for the map or reset confirmation before moving.");
    return false;
  }
  const sent = WebSocketManager.send({ type: "drive", v, omega: w });
  UIController.updateDrive(held, sent);
  return sent;
}

function updateHeld() {
  const directions = [...drivePointers.values(), ...driveKeys.values()];
  Object.keys(held).forEach(key => held[key] = Number(directions.includes(key)));
  sendDrive();
}

function releaseDrive() {
  drivePointers.clear();
  driveKeys.clear();
  if (!Object.values(held).some(Boolean)) return;
  Object.keys(held).forEach(key => held[key] = 0);
  sendDrive();
}

function sendCommand(message, successText) {
  if (!WebSocketManager.send(message)) {
    UIController.log("Commande non envoyée : reconnectez le robot, puis réessayez.");
    return false;
  }
  if (successText) UIController.log(successText);
  return true;
}

function setOperationControls(busy) {
  features?.setBusy(busy);
  for (const id of ["resetMap", "resetOdom", "loadKnownMap", "resolution", "radius", "sendGoal", "explore"]) $(id).disabled = busy;
}

function requestOperation(type, payload = {}) {
  releaseDrive();
  const requestId = window.crypto?.randomUUID?.() || `operation-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  if (!sendCommand({ ...payload, type, request_id: requestId })) return null;
  clearTimeout(operationTimer);
  pendingOperation = { type, requestId };
  setOperationControls(true);
  $("operationStatus").textContent = "Waiting for the robot to confirm…";
  operationTimer = setTimeout(() => finishOperation({ request_id: requestId, state: "error", reason: "The robot did not confirm the operation. Check the connection and retry." }), 32000);
  return requestId;
}

function finishOperation(result) {
  if (!pendingOperation || result?.request_id !== pendingOperation.requestId || result.state === "pending") return;
  clearTimeout(operationTimer);
  releaseDrive();
  const type = pendingOperation.type;
  pendingOperation = null;
  setOperationControls(false);
  if (result.state === "error") {
    pendingKnownMap = null;
    $("operationStatus").textContent = result.reason || "Operation failed. Please retry.";
    UIController.log($("operationStatus").textContent);
    return;
  }
  if (["clear_map", "reset_odom", "nav_clear_map", "configure_map"].includes(type)) {
    MapModel.clear();
    ThreeDRenderer.clearPoints();
    pendingKnownMap = null;
    knownMapLoaded = false;
    totalHits = 0;
    UIController.updateCounters({ hits: 0, occupied: 0 });
    if (type === "reset_odom") {
      RobotState.reset();
      UIController.updateRobot(RobotState.get());
      ThreeDRenderer.updateRobot(RobotState.get());
    }
    GridRenderer.render(MapModel.get(), RobotState.get());
  }
  const labels = { clear_map: "Map cleared. New scans will rebuild it.", reset_odom: "Odometry and map reset confirmed.", nav_clear_map: "Live mapping enabled.", nav_map: "Imported map localized and ready.", configure_map: "Map settings applied." };
  $("operationStatus").textContent = labels[type] || "Operation confirmed.";
  UIController.log($("operationStatus").textContent);
}

function showView(mode) {
  const is2d = mode === "2d";
  $("view2d").classList.toggle("active", is2d);
  $("view3d").classList.toggle("active", !is2d);
  $("mode2d").classList.toggle("active", is2d);
  $("mode3d").classList.toggle("active", !is2d);
  $("mode2d").setAttribute("aria-pressed", String(is2d));
  $("mode3d").setAttribute("aria-pressed", String(!is2d));
  ThreeDRenderer.setVisible(!is2d);
  if (is2d) {
    GridRenderer.resize();
    GridRenderer.render(MapModel.get(), RobotState.get());
  } else {
    ThreeDRenderer.updateRobot(RobotState.get());
    ThreeDRenderer.resize();
  }
}

function sendNavigationGoal(x, y, angleInput = $("goalYaw"), withoutAngle = false) {
  if (pendingOperation) { UIController.log("Wait for the map operation to finish before navigating."); return false; }
  const angle = withoutAngle ? "" : angleInput.value.trim();
  if (!withoutAngle && (!angleInput.validity.valid || (angle !== "" && !Number.isFinite(Number(angle))))) {
    UIController.log("Enter a finite final orientation in degrees, or leave it blank");
    angleInput.reportValidity();
    return false;
  }
  const goal = { type: "nav_goal", x, y };
  if (angle !== "") goal.yaw_deg = Number(angle);
  releaseDrive();
  if (!WebSocketManager.send(goal)) {
    UIController.log("Goal not sent: reconnect to the robot first");
    return false;
  }
  const orientation = angle === "" ? "no final turn" : `${goal.yaw_deg} degrees`;
  UIController.log(`Goal sent: (${x.toFixed(2)}, ${y.toFixed(2)}) m, ${orientation}`);
  return true;
}

function requestMapGoal(x, y) {
  if (features?.pick(x,y)) return;
  if (pendingOperation) return;
  releaseDrive();
  pendingGoal = { x, y };
  $("goalX").value = x.toFixed(2);
  $("goalY").value = y.toFixed(2);
  $("arrivalCoordinates").textContent = `X : ${x.toFixed(2)} m · Y : ${y.toFixed(2)} m`;
  $("arrivalYaw").value = $("goalYaw").value;
  $("arrivalYaw").disabled = false;
  $("arrivalWithoutAngle").checked = false;
  $("arrivalError").textContent = "";
  $("arrivalDialog").showModal();
  $("arrivalYaw").focus();
}

function processMessage(message) {
  features?.receive(message);
  if (message?.type === "command_result") {
    finishOperation(message);
    UIController.log(message.reason || "Command rejected by the robot");
    return;
  }
  if (!message || message.type !== "scan") return;
  finishOperation(message.operation);
  UIController.updateSafety(message.safety);

  if (!viewerConfigApplied && message.configuration) {
    const resolution = Number(message.configuration.map_resolution_m);
    const radius = Number(message.configuration.map_radius_m);
    if (Number.isFinite(resolution) && resolution > 0 && Number.isFinite(radius) && radius > 0) {
      MAX_LINEAR = Number(message.configuration.teleop_max_linear_speed_mps) || MAX_LINEAR;
      MAX_ANGULAR = Number(message.configuration.teleop_max_angular_speed_rps) || MAX_ANGULAR;
      ThreeDRenderer.setVoxelSize(message.configuration.point_voxel_size_m);
      MapModel.configureEvidence(message.configuration.mapping_evidence);
      $("resolution").value = resolution;
      $("resVal").textContent = resolution.toFixed(2) + " m";
      $("radius").value = radius;
      $("radiusVal").textContent = radius + " m";
      MapModel.allocate(resolution, radius);
      viewerConfigApplied = true;
      UIController.log(`[CONFIG] Viewer initialized from controller YAML (${resolution} m, radius ${radius} m)`);
    }
  }

  packetCount++;
  UIController.updateCounters({ packets: packetCount });

  if (typeof message.scanning === "boolean") {
    scanning = message.scanning;
    UIController.setScanning(scanning);
  }

  const knownMapReadyForPending = pendingKnownMap
    && message.scan_matching?.known_map_global_localized
    && message.scan_matching?.known_map_request_id === pendingKnownMap.requestId;
  const waitingForKnownMap = pendingKnownMap && !knownMapReadyForPending;
  if (message.robot && !waitingForKnownMap) {
    RobotState.update(message.robot);
    const robot = RobotState.get();

    UIController.updateRobot(robot);
    ThreeDRenderer.updateRobot(robot);
  }

  if (message.estimation_error && message.ground_truth) {
    const truth = message.ground_truth;
    const error = message.estimation_error;
    $("truthXY").textContent = `${truth.x.toFixed(2)}, ${truth.y.toFixed(2)}`;
    $("posError").textContent = error.position_m.toFixed(3);
    $("yawError").textContent = (error.yaw_rad * 180 / Math.PI).toFixed(2);
    if (message.estimator_metrics?.position_rmse_m !== null && message.estimator_metrics?.position_rmse_m !== undefined) {
      $("posRmse").textContent = Number(message.estimator_metrics.position_rmse_m).toFixed(3);
    }
  }

  if (message.navigation) {
    const nav = message.navigation;
    const safety = message.safety?.stop_active
      ? ` • SAFETY STOP: ${message.safety.reason}`
      : "";
    const state = nav.state ?? nav.status ?? "idle";
    const labels = {
      idle: "Idle", stopped: "Stopped", goal_active: "Goal accepted",
      planning: "Planning route", path_ready: "Route ready",
      following: "Moving to goal", aligning: "Adjusting final orientation",
      approaching: "Final position adjustment", braking: "Braking to a stop",
      waiting_for_pose: "Stopped: waiting for a fresh robot position",
      settling: "Confirming arrival", goal_reached: "Goal reached - stopped",
      turning_to_path: "Orientation vers le trajet",
      goal_failed: "Navigation arrêtée : aucun progrès. Vérifier les obstacles et la position du robot.",
      waiting_for_path: "Waiting for a route", waiting_for_map_or_pose: "Waiting for map and position",
      goal_outside_map: "Goal outside map - choose another position",
      outside_map_recovery: "Robot outside map - check localization",
      pathfinding_failed: "No route available - choose another goal",
      invalid_request: "Invalid goal - check coordinates and angle",
      exploration_complete: "Exploration complete"
    };
    $("navStatus").textContent = `Navigation: ${labels[state] ?? state}${safety}`;
    const signature = `${state}|${message.safety?.reason || ""}`;
    if (signature !== lastNavigationDebug) {
      UIController.log(`[NAV] ${$("navStatus").textContent}`);
      lastNavigationDebug = signature;
    }
  }

  // Do not reveal a newly uploaded map until the controller has localized the
  // first surrounding scan against it. This keeps the visual map and robot
  // pose from appearing out of sync during global relocalization.
  if (knownMapReadyForPending) {
    const loaded = pendingKnownMap;
    pendingKnownMap = null;
    MapModel.loadGrid(loaded.resolution, loaded.radius, loaded.grid);
    knownMapLoaded = true;
    if (message.robot) {
      RobotState.update(message.robot);
      const robot = RobotState.get();
      UIController.updateRobot(robot);
      ThreeDRenderer.updateRobot(robot);
    }
    UIController.log("Known map localized (>90% scan agreement); map and robot pose activated.");
  }

  // ROS publishes these endpoints in the fixed map frame. The browser may
  // convert axes at the display boundary, but must never apply robot pose to
  // map-frame points a second time.
  const points = message.points_frame === "map" && Array.isArray(message.new_points)
    ? message.new_points
    : [];
  if (message.map?.data_b64) {
    try {
      if (message.map.map_id && message.map.map_id !== activeMapId) {
        ThreeDRenderer.clearPoints();
        activeMapId = message.map.map_id;
        totalHits = 0;
      }
      const raw = atob(message.map.data_b64);
      const bytes = new Uint8Array(raw.length);
      for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);
      MapModel.loadGrid(Number(message.map.resolution), Number(message.map.radius), Array.from(new Int8Array(bytes.buffer)));
      ThreeDRenderer.setStaticMap(MapModel.get());
      $("resolution").value = message.map.resolution;
      $("resVal").textContent = Number(message.map.resolution).toFixed(2) + " m";
      $("radius").value = message.map.radius;
      $("radiusVal").textContent = message.map.radius + " m";
      knownMapLoaded = true;
    } catch (error) { UIController.log(`[MAP] decode error: ${error}`); }
  }
  if (!message.map && points.length) ThreeDRenderer.addPoints(points);
  totalHits += points.length;
  UIController.updateCounters({ hits: totalHits });
  const map = MapModel.get();
  ThreeDRenderer.setLivePoints(message.live_points ?? points);
  UIController.updateCounters({ occupied: map.occupiedCount });
  GridRenderer.render(map, RobotState.get(), message.live_points ?? points, "#53d8ff", [], "#ff4fd8");
}

function installDriveControls() {
  const releasePointer = event => {
    if (!drivePointers.delete(event.pointerId)) return;
    updateHeld();
  };
  document.querySelectorAll("#drive [data-d]").forEach(button => {
    const direction = button.dataset.d;

    const press = event => {
      if (event.button !== 0) return;
      event.preventDefault();
      button.setPointerCapture?.(event.pointerId);
      drivePointers.set(event.pointerId, direction);
      updateHeld();
    };

    button.addEventListener("pointerdown", press);
    button.addEventListener("pointerup", releasePointer);
    button.addEventListener("pointercancel", releasePointer);
    button.addEventListener("lostpointercapture", releasePointer);
    for (const type of ["keydown", "keyup"]) {
      button.addEventListener(type, event => {
        if (!["Space", "Enter"].includes(event.code)) return;
        event.preventDefault();
        const key = `button-${direction}-${event.code}`;
        if (type === "keydown") driveKeys.set(key, direction);
        else driveKeys.delete(key);
        updateHeld();
      });
    }
    button.addEventListener("blur", () => {
      for (const key of driveKeys.keys()) {
        if (key.startsWith(`button-${direction}-`)) driveKeys.delete(key);
      }
      const pressed = Number([...drivePointers.values(), ...driveKeys.values()].includes(direction));
      if (held[direction] !== pressed) updateHeld();
    });
  });
  window.addEventListener("pointerup", releasePointer);
  window.addEventListener("pointercancel", releasePointer);

  const keyMap = {
    ArrowUp: "up", w: "up", W: "up",
    ArrowDown: "down", s: "down", S: "down",
    ArrowLeft: "left", a: "left", A: "left",
    ArrowRight: "right", d: "right", D: "right"
  };

  window.addEventListener("keydown", event => {
    if ($("arrivalDialog").open) return;
    if (event.target.closest?.("input, textarea, select, [contenteditable='true']")) return;
    if (event.code === "Space") {
      if (event.target.closest?.("button")) return;
      event.preventDefault();

      if (!event.repeat) {
        sendCommand({ type: "toggle_scan" }, "Changement d’état du LiDAR demandé");
      }
      return;
    }

    const direction = keyMap[event.key];
    if (direction) {
      driveKeys.set(event.code || event.key.toLowerCase(), direction);
      updateHeld();
      event.preventDefault();
    }
  });

  window.addEventListener("keyup", event => {
    const direction = keyMap[event.key];
    if (direction && driveKeys.delete(event.code || event.key.toLowerCase())) {
      updateHeld();
      event.preventDefault();
    }
  });
  window.addEventListener("blur", releaseDrive);
  window.addEventListener("pagehide", releaseDrive);
  document.addEventListener?.("visibilitychange", () => {
    if (document.hidden) releaseDrive();
  });

  // Refresh only while a UI direction is actively held. Sending an idle
  // zero-velocity command continuously would overwrite commands from another
  // controller or autonomy layer connected to the same robot interface.
  setInterval(() => {
    if (Object.values(held).some(Boolean)) sendDrive();
  }, 100);
}

function installUI() {
  $("arrivalForm").onsubmit = event => {
    event.preventDefault();
    if (!pendingGoal) return;
    const withoutAngle = $("arrivalWithoutAngle").checked;
    const angle = $("arrivalYaw");
    if (!withoutAngle && (angle.value.trim() === "" || !angle.validity.valid || !Number.isFinite(Number(angle.value)))) {
      $("arrivalError").textContent = "Saisissez un angle ou choisissez « Sans orientation finale ».";
      angle.focus();
      return;
    }
    if (!sendNavigationGoal(pendingGoal.x, pendingGoal.y, angle, withoutAngle)) {
      $("arrivalError").textContent = "Objectif non envoyé : reconnectez le robot, puis réessayez.";
      return;
    }
    $("goalYaw").value = withoutAngle ? "" : angle.value;
    pendingGoal = null;
    $("arrivalDialog").close();
  };
  $("arrivalCancel").onclick = () => $("arrivalDialog").close();
  $("arrivalDialog").addEventListener("close", () => { pendingGoal = null; });
  $("arrivalWithoutAngle").onchange = () => {
    $("arrivalYaw").disabled = $("arrivalWithoutAngle").checked;
    $("arrivalError").textContent = "";
  };
  $("arrivalYaw").oninput = () => { $("arrivalError").textContent = ""; };
  $("stop").onclick = () => {
    releaseDrive();
    sendCommand({ type: "stop" }, "Arrêt du robot demandé");
  };

  $("scan").onclick = () => {
    sendCommand({
      type: "set_scan",
      enabled: !scanning
    });
  };

  $("resetMap").onclick = () => {
    requestOperation("clear_map");
  };

  $("resetOdom").onclick = () => {
    requestOperation("reset_odom");
  };

  $("resolution").oninput = event => {
    const value = Number(event.target.value);
    $("resVal").textContent = value.toFixed(2) + " m";

  };

  $("radius").oninput = event => {
    const value = Number(event.target.value);
    $("radiusVal").textContent = value + " m";

  };
  const applyMapSettings = () => requestOperation("configure_map", { resolution: Number($("resolution").value), radius: Number($("radius").value) });
  $("resolution").onchange = applyMapSettings;
  $("radius").onchange = applyMapSettings;

  $("save").onclick = () => {
    MapExporter.exportPNG(MapModel.get(),features.getConfig()).catch(error => UIController.log(error.message));
  };
  $("saveBundle").onclick = () => MapExporter.exportJSON(MapModel.get(),features.getConfig());

  $("explore").onclick = () => {
    releaseDrive();
    if (sendCommand({ type: "nav_explore" }, "Exploration demandée")) {
      $("navStatus").textContent = "Exploration demandée : attente du robot…";
    }
  };

  $("navStop").onclick = () => {
    releaseDrive();
    const sent = WebSocketManager.send({ type: "nav_stop" });
    UIController.log(sent ? "Navigation stop requested" : "Stop not sent: robot disconnected");
  };

  $("sendGoal").onclick = () => {
    const x = Number($("goalX").value);
    const y = Number($("goalY").value);
    if ($("goalX").value.trim() === "" || $("goalY").value.trim() === "" || !Number.isFinite(x) || !Number.isFinite(y)) {
      UIController.log("Enter finite goal x and y coordinates");
      return;
    }
    sendNavigationGoal(x, y);
  };

  $("loadKnownMap").onclick = async () => {
    const file = $("knownMapFile").files?.[0];
    if (!file) {
      UIController.log("Choose a known-map image first");
      return;
    }
    let resolution = Number($("knownMapResolution").value);
    let radius = Number($("knownMapRadius").value);
    if (!Number.isFinite(resolution) || !Number.isFinite(radius) || !(resolution > 0) || !(radius > 0) || Math.ceil(2 * radius / resolution) ** 2 > 1000000) {
      UIController.log("Known-map resolution and radius must be positive");
      return;
    }
    const url = URL.createObjectURL(file);
    try {
      if (file.name.toLowerCase().endsWith('.json') || file.type === 'application/json') {
        const loaded = validateBundle(JSON.parse(await file.text()));
        const requestId = requestOperation('nav_map',{mode:'known_image',...loaded});
        if(requestId) pendingKnownMap = {requestId,...loaded};
        return;
      }
      const metadata = typeof file.arrayBuffer === 'function' ? readPNGMetadata(new Uint8Array(await file.arrayBuffer())) : null;
      let navigation_config = { zones:[],base:null };
      if(metadata) {
        if(metadata.format !== 'fabtino-map' || metadata.version !== 1) throw new Error('Unsupported map metadata.');
        resolution=metadata.resolution; radius=metadata.radius; navigation_config=validateNavigation(metadata.navigation_config);
        if(!Number.isFinite(resolution) || !Number.isFinite(radius) || resolution<=0 || radius<=0 || Math.ceil(2*radius/resolution)**2>1000000) throw new Error('Invalid saved map scale.');
      }
      const image = await new Promise((resolve, reject) => {
        const img = new Image();
        img.onload = () => resolve(img);
        img.onerror = reject;
        img.src = url;
      });
      const size = Math.ceil(2 * radius / resolution);
      if (image.width !== image.height) throw new Error("Choose a square occupancy-map image and enter its original scale.");
      const canvas = document.createElement("canvas");
      canvas.width = size;
      canvas.height = size;
      const ctx = canvas.getContext("2d", { willReadFrequently: true });
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(image, 0, 0, size, size);
      const pixels = ctx.getImageData(0, 0, size, size).data;
      const grid = new Array(size * size).fill(-1);
      for (let py = 0; py < size; py++) {
        for (let px = 0; px < size; px++) {
          const p = (py * size + px) * 4;
          const luminance = (pixels[p] + pixels[p + 1] + pixels[p + 2]) / 3;
          const gy = size - 1 - py;
          grid[gy * size + px] = pixels[p + 3] < 128 ? -1 : luminance < 100 ? 100 : luminance > 200 ? 0 : -1;
        }
      }
      const requestId = requestOperation("nav_map", { mode: "known_image", resolution, radius, grid, navigation_config });
      if (!requestId) return;
      pendingKnownMap = { requestId, resolution, radius, grid };
      UIController.log("Known map sent. Collecting surrounding LiDAR scan for global relocalization…");
    } catch (error) {
      UIController.log(`Known-map load failed: ${error.message || error}`);
    } finally {
      URL.revokeObjectURL(url);
    }
  };

  $("liveMapMode").onclick = () => {
    requestOperation("nav_clear_map");
  };

  $("mode2d").onclick = () => {
    requestedView = "2d";
    $("viewStatus").textContent = "";
    showView("2d");
  };

  $("mode3d").onclick = async () => {
    requestedView = "3d";
    $("viewStatus").textContent = "Chargement de la vue 3D…";
    const ready = await ThreeDRenderer.init($("threeCanvas"));
    if (requestedView !== "3d") return;
    if (ready === false) {
      $("viewStatus").textContent = "Vue 3D indisponible : vérifiez WebGL dans votre navigateur.";
      UIController.log($("viewStatus").textContent);
      showView("2d");
      return;
    }
    $("viewStatus").textContent = "";
    showView("3d");
  };
}

function init() {
  MapModel.allocate(0.05, 20);

  GridRenderer.init($("mapCanvas"));
  GridRenderer.setClickHandler(requestMapGoal);

  // Compatibility bridge used only by the GridRenderer's pointer handler.
  window.__mapViewerMapState = () => MapModel.get();

  WebSocketManager.on("status", status => {
    UIController.setStatus(status.connected, status.text);
    features?.connection(status.connected);
  });

  WebSocketManager.on("open", () => {
    UIController.log("WebSocket connected");
  });

  WebSocketManager.on("close", () => {
    releaseDrive();
    if (pendingOperation) finishOperation({ request_id: pendingOperation.requestId, state: "error", reason: "Connection lost before confirmation. Reconnect and retry." });
    UIController.log("WebSocket closed; reconnecting...");
  });

  WebSocketManager.on("parse-error", () => {
    UIController.log("Ignored malformed WebSocket JSON");
  });

  WebSocketManager.on("message", processMessage);

  installDriveControls();
  installUI();
  features = createNavigationFeatures({ $, send: message => sendCommand(message), log:UIController.log, robot:RobotState.get, releaseDrive,
    overlays:value => { GridRenderer.setOverlays(value);ThreeDRenderer.setOverlays(value); }, pickView:() => $("mode2d").click() });

  window.addEventListener("resize", () => {
    GridRenderer.resize();
    GridRenderer.render(MapModel.get(), RobotState.get());
    ThreeDRenderer.resize();
  });

  ThreeDRenderer.setVisible(false);
  ThreeDRenderer.animate();
  GridRenderer.render(MapModel.get(), RobotState.get());

  WebSocketManager.connect();
}

init();
