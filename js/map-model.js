export const MapModel = (() => {
  let resolution = 0.05;
  let radius = 20;
  let size = 0;
  let grid = null;
  let evidence = null;
  let occupiedCount = 0;
  let revision = 0;
  let changedCells = new Set();
  let freeEvidenceStep = 1;
  let occupiedEvidenceStep = 2;
  let freeThreshold = -2;
  let occupiedThreshold = 2;

  function allocate(newResolution = resolution, newRadius = radius) {
    revision++;
    resolution = Number(newResolution);
    radius = Number(newRadius);
    size = Math.ceil((radius * 2) / resolution);
    grid = new Int8Array(size * size);
    grid.fill(-1);
    // Signed evidence prevents one noisy return from permanently deciding a
    // cell. The public grid remains the compact -1/0/100 protocol used by the
    // renderer and exporter.
    evidence = new Int16Array(size * size);
    occupiedCount = 0;
    changedCells = new Set();
  }

  function cellIndex(x, y) {
    const gx = Math.floor((x + radius) / resolution);
    const gy = Math.floor((y + radius) / resolution);
    if (gx < 0 || gy < 0 || gx >= size || gy >= size) return -1;
    return gy * size + gx;
  }

  function setFree(x, y) {
    const i = cellIndex(x, y);
    if (i < 0) return;
    evidence[i] = Math.max(-12, evidence[i] - freeEvidenceStep);
    const next = evidence[i] <= freeThreshold ? 0 : -1;
    if (grid[i] !== next) {
      if (grid[i] === 100) occupiedCount--;
      if (next === 100) occupiedCount++;
      grid[i] = next;
      changedCells.add(i);
      revision++;
    }
  }

  function setOccupied(x, y) {
    const i = cellIndex(x, y);
    if (i < 0) return;

    evidence[i] = Math.min(12, evidence[i] + occupiedEvidenceStep);
    const next = evidence[i] >= occupiedThreshold ? 100 : -1;
    if (grid[i] !== next) {
      if (grid[i] === 100) occupiedCount--;
      if (next === 100) occupiedCount++;
      grid[i] = next;
      changedCells.add(i);
      revision++;
    }
  }

  function rayTrace(x0, y0, x1, y1) {
    const dx = x1 - x0;
    const dy = y1 - y0;
    const steps = Math.max(1, Math.ceil(Math.hypot(dx, dy) / (resolution * 0.7)));

    for (let i = 0; i < steps; i++) {
      const t = i / steps;
      setFree(x0 + dx * t, y0 + dy * t);
    }
    setOccupied(x1, y1);
  }

  function consumeChanges() {
    const result = changedCells;
    changedCells = new Set();
    return result;
  }

  function clear() {
    allocate(resolution, radius);
  }

  function loadGrid(newResolution, newRadius, values) {
    const r=Number(newResolution), extent=Number(newRadius);
    const expected=Math.ceil(2*extent/r)**2;
    if (!Number.isFinite(r) || !Number.isFinite(extent) || r<=0 || extent<=0 || expected>1000000
        || !Array.isArray(values) || values.length!==expected || values.some(value=>![-1,0,100].includes(value))) {
      throw new Error("Invalid occupancy map dimensions or cells");
    }
    allocate(r, extent);
    grid.set(values);
    occupiedCount = values.reduce((count, value) => count + (Number(value) === 100 ? 1 : 0), 0);
    return true;
  }

  function configureEvidence(config = {}) {
    freeEvidenceStep = Math.max(1, Number(config.free_evidence_step) || 1);
    occupiedEvidenceStep = Math.max(1, Number(config.occupied_evidence_step) || 1);
    freeThreshold = Number(config.free_threshold ?? -2);
    occupiedThreshold = Number(config.occupied_threshold ?? 2);
  }

  function get() {
    return {
      grid,
      size,
      resolution,
      radius,
      occupiedCount
      ,revision
    };
  }

  return {
    allocate,
    setFree,
    setOccupied,
    rayTrace,
    consumeChanges,
    clear,
    loadGrid,
    configureEvidence,
    get
  };
})();
