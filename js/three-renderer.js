// Load the bundled Three.js files only when requested. Missing WebGL must
// leave the 2D map and teleoperation controls available.
let THREE = null;
let OrbitControls = null;

export const ThreeDRenderer = (() => {
  const MAX_POINTS = 250000;
  let canvas, renderer, scene, camera, controls;
  let pointGeometry, pointCloud;
  let robotGroup;
  let pointIndex = 0;
  let pointCount = 0;
  let voxelSize = 0.05;
  const voxelIndex = new Map();
  const voxelKeys = new Array(MAX_POINTS);
  let visible = false;
  let initialization = null;
  let latestRobot = null;
  let pendingPoints = [];
  let overlayGroup=null, latestOverlays={zones:[],base:null}, overlayKey='';
  let liveGeometry=null,latestLive=[];

  const positions = new Float32Array(MAX_POINTS * 3);
  const colors = new Float32Array(MAX_POINTS * 3);

  function init(element) {
    canvas = element;
    if (!initialization) initialization = initialize();
    return initialization;
  }

  async function initialize() {
    try {
      const modules = await Promise.all([
        import("three"),
        import("three/addons/controls/OrbitControls.js")
      ]);
      THREE = modules[0];
      OrbitControls = modules[1].OrbitControls;
      scene = new THREE.Scene();
      scene.background = new THREE.Color(0x080b10);

      camera = new THREE.PerspectiveCamera(55, 1, 0.05, 100);
      camera.position.set(6, 5, 6);

      renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true
      });
      renderer.setPixelRatio(devicePixelRatio || 1);

      controls = new OrbitControls(camera, canvas);
      controls.enableDamping = true;
      controls.target.set(0, 0, 0);

      scene.add(new THREE.GridHelper(40, 40, 0x2a3742, 0x18232d));
      scene.add(new THREE.AmbientLight(0xffffff, 0.7));

      pointGeometry = new THREE.BufferGeometry();
      pointGeometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3)
      );
      pointGeometry.setAttribute(
        "color",
        new THREE.BufferAttribute(colors, 3)
      );
      pointGeometry.setDrawRange(0, 0);

      pointCloud = new THREE.Points(
        pointGeometry,
        new THREE.PointsMaterial({
          size: 0.055,
          vertexColors: true,
          sizeAttenuation: true
        })
      );
      scene.add(pointCloud);
      liveGeometry=new THREE.BufferGeometry();
      scene.add(new THREE.Points(liveGeometry,new THREE.PointsMaterial({color:0x53d8ff,size:.06,sizeAttenuation:true})));
      setLivePoints(latestLive);

      robotGroup = new THREE.Group();
      scene.add(robotGroup);
      overlayGroup=new THREE.Group();scene.add(overlayGroup);
      setOverlays(latestOverlays);

      const body = new THREE.Mesh(
        new THREE.BoxGeometry(0.55, 0.28, 0.50),
        new THREE.MeshBasicMaterial({
          color: 0x38f28a,
          wireframe: true
        })
      );
      robotGroup.add(body);

      const arrow = new THREE.Mesh(
        new THREE.ConeGeometry(0.055, 0.14, 8),
        new THREE.MeshBasicMaterial({ color: 0xff5b5b })
      );
      arrow.rotation.z = -Math.PI / 2;
      arrow.position.x = 0.31;
      robotGroup.add(arrow);

      robotGroup.add(new THREE.AxesHelper(0.45));
      if (latestRobot) updateRobot(latestRobot);
      addPoints(pendingPoints);
      pendingPoints = [];
      resize();
      return true;
    } catch (error) {
      console.warn("3D renderer unavailable; continuing with 2D viewer:", error);
      renderer?.dispose();
      renderer = null;
      controls = null;
      pointGeometry = null;
      robotGroup = null;
      return false;
    }
  }

  function addPoints(worldPoints) {
    if (!Array.isArray(worldPoints)) return;
    if (!pointGeometry) {
      pendingPoints = pendingPoints.concat(worldPoints.slice(-MAX_POINTS)).slice(-MAX_POINTS);
      return;
    }

    for (const point of worldPoints) {
      if (!Array.isArray(point) || point.length < 2) continue;

      const wx = Number(point[0]);
      const wy = Number(point[1]);
      const wzRaw = Number(point[2]);
      const wz = Number.isFinite(wzRaw) ? wzRaw : 0.285;

      if (!Number.isFinite(wx) || !Number.isFinite(wy)) continue;

      const vx = Math.floor(wx / voxelSize);
      const vy = Math.floor(wz / voxelSize);
      const vz = Math.floor(-wy / voxelSize);
      const key = `${vx}:${vy}:${vz}`;
      let i = voxelIndex.get(key);
      if (i === undefined) {
        i = pointIndex % MAX_POINTS;
        if (pointCount >= MAX_POINTS) voxelIndex.delete(voxelKeys[i]);
        else pointCount++;
        voxelKeys[i] = key;
        voxelIndex.set(key, i);
        pointIndex++;
      }

      positions[i * 3] = wx;
      positions[i * 3 + 1] = wz;
      positions[i * 3 + 2] = -wy;

      let color;
      if (wz < 0.10) color = new THREE.Color(0xff5252);
      else if (wz < 0.50) color = new THREE.Color(0xf4d35e);
      else color = new THREE.Color(0x48a8ff);

      colors[i * 3] = color.r;
      colors[i * 3 + 1] = color.g;
      colors[i * 3 + 2] = color.b;

    }

    pointGeometry.setDrawRange(
      0,
      pointCount
    );
    pointGeometry.attributes.position.needsUpdate = true;
    pointGeometry.attributes.color.needsUpdate = true;
  }

  function updateRobot(robot) {
    latestRobot = { ...robot };
    if (!robotGroup) return;

    robotGroup.position.set(robot.x, 0, -robot.y);
    // Map +Y is Three.js -Z: a positive yaw turns local +X toward -Z.
    robotGroup.rotation.set(0, robot.yaw, 0);
  }

  function resize() {
    if (!renderer || !canvas) return;

    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;

    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  function setVisible(value) {
    visible = !!value;
  }

  function animate() {
    requestAnimationFrame(animate);
    if (!visible || !renderer || !controls) return;

    controls.update();
    renderer.render(scene, camera);
  }

  function clearPoints() {
    pendingPoints = [];
    if (!pointGeometry) return;
    pointIndex = 0;
    pointCount = 0;
    voxelIndex.clear();
    pointGeometry.setDrawRange(0, 0);
  }

  function setVoxelSize(value) {
    const size = Number(value);
    if (Number.isFinite(size) && size > 0) voxelSize = size;
  }

  function setStaticMap(map) {
    // Replace from the authoritative static raster so removed objects leave no 3D ghosts.
    clearPoints();
    const points=[];
    const stride=Math.max(1,Math.ceil((map.occupiedCount || 0)/MAX_POINTS));let occupied=0;
    for(let y=0;y<map.size;y++) for(let x=0;x<map.size;x++) {
      if(map.grid[y*map.size+x] === 100 && occupied++%stride===0) points.push([(x+.5)*map.resolution-map.radius,(y+.5)*map.resolution-map.radius,0]);
    }
    addPoints(points);
  }

  function setOverlays(value) {
    latestOverlays=value;
    if(!overlayGroup) return;
    const key=JSON.stringify(value);
    if(key===overlayKey) return; overlayKey=key;
    for(const item of [...overlayGroup.children]) { overlayGroup.remove(item);item.geometry.dispose();item.material.dispose(); }
    for(const zone of value.zones || []) {
      const geometry=new THREE.RingGeometry(Math.max(.001,zone.radius-.015),zone.radius+.015,64);
      const ring=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({color:0xff5266,side:THREE.DoubleSide}));
      ring.rotation.x=-Math.PI/2;ring.position.set(zone.x,.02,-zone.y);overlayGroup.add(ring);
    }
    for(const [point,color] of [[value.base,0x39d9ff],[value.target,0xe7eef5]]) {
      if(!point) continue;
      const marker=new THREE.Mesh(new THREE.RingGeometry(.08,.12,24),new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide}));
      marker.rotation.x=-Math.PI/2;marker.position.set(point.x,.03,-point.y);overlayGroup.add(marker);
    }
  }

  function setLivePoints(points) {
    latestLive=Array.isArray(points)?points:[];
    if(!liveGeometry)return;
    const finite=latestLive.filter(p=>Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1])).slice(0,20000);
    const coordinates=new Float32Array(finite.length*3);
    finite.forEach((p,i)=>coordinates.set([p[0],Number.isFinite(p[2])?p[2]:0,-p[1]],i*3));
    liveGeometry.setAttribute('position',new THREE.BufferAttribute(coordinates,3));liveGeometry.computeBoundingSphere();
  }

  return {
    init,
    addPoints,
    updateRobot,
    resize,
    setVisible,
    clearPoints,
    setVoxelSize,
    setStaticMap,
    setOverlays,
    setLivePoints,
    animate
  };
})();
