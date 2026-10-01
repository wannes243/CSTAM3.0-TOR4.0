export const UIController = (() => {
  const $ = id => document.getElementById(id);

  function log(message) {
    const element = document.createElement("div");
    element.textContent =
      `[${new Date().toLocaleTimeString()}] ${message}`;

    $("log").appendChild(element);
    $("log").scrollTop = $("log").scrollHeight;

    while ($("log").children.length > 30) {
      $("log").firstChild.remove();
    }
  }

  function setStatus(connected, text) {
    $("dot").classList.toggle("on", connected);
    $("status").textContent = text;
    if (!connected) $("driveStatus").textContent = "Robot déconnecté : commandes indisponibles.";
    else $("driveStatus").textContent = "Prêt : maintenez une direction pour déplacer le robot.";
  }

  function updateDrive(held, sent, reason = "") {
    document.querySelectorAll("#drive [data-d]").forEach(button => {
      button.classList.toggle("active", Boolean(held[button.dataset.d]) && sent);
    });
    if (!sent) {
      $("driveStatus").textContent = reason || "Commande non envoyée : robot déconnecté.";
      return;
    }
    const directions = [];
    if (held.up !== held.down) directions.push(held.up ? "Avance" : "Recule");
    if (held.left !== held.right) directions.push(held.left ? "Tourne à gauche" : "Tourne à droite");
    $("driveStatus").textContent = directions.length ? `Commande : ${directions.join(" et ")}` : "Commande d’arrêt envoyée.";
  }

  function updateSafety(safety = {}) {
    const reasons = {
      sensor_not_ready: "En attente des capteurs du robot.",
      sensor_timeout: "Données LiDAR interrompues : relancez le scan.",
      localization_timeout: "Position du robot indisponible.",
      emergency_stop: "Arrêt d’urgence actif."
    };
    let text = "";
    if (safety?.stop_active) {
      const reason = safety.reason || "";
      if (reason.startsWith("rotation_obstacle=")) text = "Rotation bloquée : un obstacle est trop proche du robot.";
      else if (reason.startsWith("front_obstacle=")) text = "Avance bloquée : un obstacle se trouve devant le robot.";
      else if (reason.startsWith("rear_obstacle=")) text = "Recul bloqué : un obstacle se trouve derrière le robot. Avancez pour vous éloigner.";
      else if (reason.startsWith("denied_zone=")) text = `Navigation stopped at denied zone: ${reason.slice(12)}.`;
      else text = reasons[reason] || "Robot arrêté : en attente d’une commande récente.";
    }
    $("safetyStatus").textContent = text;
    $("safetyStatus").hidden = !text;
  }

  function setScanning(scanning) {
    $("scanBadge").textContent =
      scanning ? "SCAN ON" : "SCAN OFF";

    $("scan").textContent =
      scanning ? "Stop LiDAR Scan" : "Start 360° LiDAR Scan";

    $("scan").classList.toggle("active", scanning);
  }

  function updateRobot(robot) {
    $("x").textContent = robot.x.toFixed(2);
    $("y").textContent = robot.y.toFixed(2);
    $("yaw").textContent =
      (robot.yaw * 180 / Math.PI).toFixed(1);
  }

  function updateCounters({ hits, occupied, packets }) {
    if (hits !== undefined) {
      $("hits").textContent = Number(hits).toLocaleString();
    }
    if (occupied !== undefined) {
      $("occ").textContent = Number(occupied).toLocaleString();
    }
    if (packets !== undefined) {
      $("packets").textContent = Number(packets).toLocaleString();
    }
  }

  return {
    $,
    log,
    setStatus,
    updateDrive,
    updateSafety,
    setScanning,
    updateRobot,
    updateCounters
  };
})();
