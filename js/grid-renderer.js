export const GridRenderer = (() => {
  let canvas = null;
  let ctx = null;
  let zoom = 1;
  let panX = 0;
  let panY = 0;
  let dragging = false;
  let lastPX = 0;
  let lastPY = 0;
  let downPX = 0;
  let downPY = 0;
  let clickHandler = null;
  let needsFullRedraw = true;
  let lastFrame = null;
  let overlays = { zones: [], base: null };
  let raster = null, rasterGrid = null, rasterRevision = null, overlaySignature = '';

  function redraw() {
    if (lastFrame) render(...lastFrame);
  }

  function init(element) {
    canvas = element;
    ctx = canvas.getContext("2d");
    resize();
    installInput();
  }

  function resize() {
    if (!canvas) return;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const dpr = devicePixelRatio || 1;
    if (!w || !h) return;

    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    needsFullRedraw = true;
    redraw();
  }

  function transform(x, y, map) {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const scale = Math.min(w, h) / (2 * map.radius) * zoom;

    return [
      w / 2 + (x - panX) * scale,
      h / 2 - (y - panY) * scale
    ];
  }

  function render(map, robot, livePoints = [], liveColor = "#53d8ff",
                  secondaryPoints = [], secondaryColor = "#ff4fd8") {
    lastFrame = [map, robot, livePoints, liveColor, secondaryPoints, secondaryColor];
    if (!canvas || !ctx) return;

    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;

    const scale = Math.min(w, h) / (2 * map.radius) * zoom;
    const ox = w / 2 - panX * scale;
    const oy = h / 2 + panY * scale;
    const step = Math.max(1, Math.floor(map.size / 700));
    const px = Math.max(1, map.resolution * scale);

    ctx.fillStyle = "#070b10";
    ctx.fillRect(0, 0, w, h);

    if (typeof document !== 'undefined' && ctx.drawImage) {
      if (!raster || rasterGrid !== map.grid || rasterRevision !== map.revision) {
        raster = document.createElement('canvas');raster.width=raster.height=map.size;
        const rasterContext=raster.getContext('2d'),image=rasterContext.createImageData(map.size,map.size);
        for(let y=0;y<map.size;y++) for(let x=0;x<map.size;x++) {
          const value=map.grid[y*map.size+x],i=((map.size-1-y)*map.size+x)*4;
          if(value>=0)image.data.set(value===100?[244,211,94,255]:[36,49,61,255],i);
        }
        rasterContext.putImageData(image,0,0);rasterGrid=map.grid;rasterRevision=map.revision;
      }
      ctx.imageSmoothingEnabled=false;
      ctx.drawImage(raster,ox-map.radius*scale,oy-(map.size*map.resolution-map.radius)*scale,map.size*map.resolution*scale,map.size*map.resolution*scale);
    } else for (let gy = 0; gy < map.size; gy += step) {
      for (let gx = 0; gx < map.size; gx += step) {
        const value = map.grid[gy * map.size + gx];
        if (value < 0) continue;

        // Grid values represent cells, so draw each cell from its lower-left
        // corner. Point and robot coordinates use the same world frame.
        const x = gx * map.resolution - map.radius;
        const y = gy * map.resolution - map.radius;

        ctx.fillStyle = value === 100 ? "#f4d35e" : "#24313d";
        ctx.fillRect(
          ox + x * scale,
          oy - (y + map.resolution * step) * scale,
          Math.max(1, px * step),
          Math.max(1, px * step)
        );
      }
    }

    function drawPoints(points, color) {
      if (!Array.isArray(points)) return;
      ctx.fillStyle = color;
      for (const point of points) {
        if (!Array.isArray(point) || point.length < 2) continue;
        const x = Number(point[0]);
        const y = Number(point[1]);
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        const [pxPoint, pyPoint] = transform(x, y, map);
        ctx.fillRect(pxPoint - 2, pyPoint - 2, 4, 4);
      }
    }

    // Current LiDAR is always cyan. In known-map mode, residual/dynamic
    // returns are drawn on top in magenta so both layers remain observable.
    drawPoints(livePoints, liveColor);
    drawPoints(secondaryPoints, secondaryColor);

    ctx.save();
    for (const zone of overlays.zones || []) {
      const [x,y] = transform(zone.x,zone.y,map);
      ctx.beginPath(); ctx.arc(x,y,zone.radius*scale,0,2*Math.PI);
      ctx.fillStyle = '#ff526626'; ctx.fill(); ctx.strokeStyle = '#ff5266'; ctx.lineWidth=2; ctx.stroke();
      ctx.fillStyle = '#ffacb6'; ctx.font='12px Segoe UI'; ctx.fillText(zone.name,x+zone.radius*scale+5,y);
    }
    for (const [point,label,color] of [[overlays.base,'BASE','#39d9ff'],[overlays.target,'STOP','#e7eef5']]) {
      if (!point) continue;
      const [x,y] = transform(point.x,point.y,map); ctx.strokeStyle=color; ctx.lineWidth=2;
      ctx.strokeRect(x-5,y-5,10,10); ctx.fillStyle=color; ctx.fillText(label,x+9,y-9);
    }
    ctx.restore();
    drawRobot(robot, map);
    needsFullRedraw = false;
  }

  function drawRobot(robot, map) {
    const [rx, ry] = transform(robot.x, robot.y, map);

    ctx.save();
    ctx.translate(rx, ry);
    // Map coordinates use +X forward and +Y left. Canvas Y is downward, so
    // the visual heading is the negative mathematical yaw.
    ctx.rotate(-robot.yaw);

    ctx.fillStyle = "#38f28a";
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(-10, -8);
    ctx.lineTo(-10, 8);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();
  }

  function zoomBy(factor) {
    zoom = Math.max(0.4, Math.min(8, zoom * factor));
    needsFullRedraw = true;
    redraw();
  }

  function resetView() {
    zoom = 1;
    panX = 0;
    panY = 0;
    needsFullRedraw = true;
    redraw();
  }

  function installInput() {
    canvas.addEventListener("wheel", event => {
      event.preventDefault();
      zoomBy(event.deltaY < 0 ? 1.12 : 0.89);
      needsFullRedraw = true;
    }, { passive: false });

    canvas.addEventListener("pointerdown", event => {
      dragging = true;
      lastPX = event.clientX;
      lastPY = event.clientY;
      downPX = event.clientX;
      downPY = event.clientY;
      canvas.setPointerCapture(event.pointerId);
    });

    canvas.addEventListener("pointermove", event => {
      if (!dragging) return;

      // Do not turn the small movement of a normal click into a persistent
      // camera pan. This was especially noticeable as a small vertical shift
      // after selecting a destination on the map.
      if (Math.hypot(event.clientX - downPX, event.clientY - downPY) < 5) return;

      const map = window.__mapViewerMapState?.();
      if (!map) return;

      const scale = Math.min(
        canvas.clientWidth,
        canvas.clientHeight
      ) / (2 * map.radius) * zoom;

      panX -= (event.clientX - lastPX) / scale;
      panY += (event.clientY - lastPY) / scale;

      lastPX = event.clientX;
      lastPY = event.clientY;
      needsFullRedraw = true;
      redraw();
    });

    canvas.addEventListener("pointerup", event => {
      if (dragging && Math.hypot(event.clientX - downPX, event.clientY - downPY) < 5 && clickHandler) {
        const map = window.__mapViewerMapState?.();
        if (map) {
          const rect = canvas.getBoundingClientRect();
          const scale = Math.min(canvas.clientWidth, canvas.clientHeight) / (2 * map.radius) * zoom;
          const px = event.clientX - rect.left;
          const py = event.clientY - rect.top;
          clickHandler(panX + (px - canvas.clientWidth / 2) / scale,
            panY - (py - canvas.clientHeight / 2) / scale);
        }
      }
      dragging = false;
    });
    canvas.addEventListener("pointercancel", () => dragging = false);
  }

  function getView() {
    return { zoom, panX, panY };
  }

  function needsRender() {
    return needsFullRedraw;
  }

  function setClickHandler(handler) {
    clickHandler = handler;
  }

  return {
    init,
    resize,
    render,
    zoomBy,
    resetView,
    getView,
    needsRender,
    setClickHandler
    ,setOverlays(value) { const signature=JSON.stringify(value);if(signature===overlaySignature)return;overlaySignature=signature;overlays=value;redraw(); }
  };
})();
