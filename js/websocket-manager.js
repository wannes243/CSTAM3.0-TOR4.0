export const WebSocketManager = (() => {
  let socket = null;
  let reconnectTimer = null;
  let manuallyClosed = false;
  const listeners = new Map();

  function emit(type, payload) {
    (listeners.get(type) || []).forEach(fn => fn(payload));
  }

  function on(type, fn) {
    if (!listeners.has(type)) listeners.set(type, new Set());
    listeners.get(type).add(fn);
    return () => listeners.get(type)?.delete(fn);
  }

  function connect() {
    manuallyClosed = false;
    emit("status", { connected: false, text: "Connecting…" });

    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const url = `${protocol}://${location.hostname || "localhost"}:8765`;
    socket = new WebSocket(url);

    socket.addEventListener("open", () => {
      emit("status", { connected: true, text: "Connected" });
      emit("open");
    });

    socket.addEventListener("close", () => {
      emit("status", { connected: false, text: "Disconnected" });
      emit("close");
      if (!manuallyClosed) scheduleReconnect();
    });

    socket.addEventListener("error", error => {
      emit("status", { connected: false, text: "WebSocket error" });
      emit("error", error);
    });

    socket.addEventListener("message", event => {
      let message;
      try {
        message = JSON.parse(event.data);
      } catch (error) {
        emit("parse-error", error);
        return;
      }
      emit("message", message);
    });
  }

  function scheduleReconnect() {
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connect, 1500);
  }

  function send(message) {
    if (socket?.readyState === WebSocket.OPEN) {
      try {
        socket.send(JSON.stringify(message));
        return true;
      } catch {
        return false;
      }
    }
    return false;
  }

  function close() {
    manuallyClosed = true;
    clearTimeout(reconnectTimer);
    socket?.close();
  }

  return { connect, close, send, on };
})();
