import { validateNavigation } from './map-exporter.js?v=delivery-v8';

export function createNavigationFeatures({ $, send, log, robot, releaseDrive, overlays, pickView }) {
  let config = { zones: [], base: null }, tasks = [], placement = null, pending = null;
  let connected = false, mapBusy = false, delivery = { state: 'idle' }, timer;
  let configReady = false;
  let renderKey = '';
  const active = () => ['planning','approaching','waiting','returning'].includes(delivery.state);
  const uuid = () => crypto.randomUUID();
  const status = text => { $('zoneStatus').textContent = text; };
  function cancelPlacement() {
    placement = null; $('placementHint').hidden = true; $('mapCanvas').style.cursor = '';
  }
  function render() {
    const label = { idle:'Delivery idle', planning:'Planning delivery', approaching:'Approaching destination', waiting:'Waiting at destination', returning:'Returning to base', complete:'Delivery complete', cancelled:'Delivery cancelled', failed:'Delivery stopped' }[delivery.state] || delivery.state;
    $('deliveryStatus').textContent = label + (delivery.tasks?.length ? ` · ${delivery.completed ?? 0}/${delivery.tasks.length} tasks done` : '') + (delivery.state === 'waiting' ? ` · ${Math.ceil(delivery.remaining_s ?? 0)}s remaining` : '') + (delivery.reason ? ` · ${delivery.reason}` : '');
    overlays({ ...config, target: active() ? delivery.target : null });
    const key = JSON.stringify([config,tasks,connected,configReady,mapBusy,pending,active()]);
    if(key === renderKey) return;
    renderKey = key;
    const selected = $('taskZone').value;
    $('zoneList').replaceChildren(); $('taskZone').replaceChildren();
    for (const zone of config.zones) {
      const row = document.createElement('div'); row.className = 'feature-item';
      const text = document.createElement('span'); text.textContent = `${zone.name} · r ${zone.radius.toFixed(2)} m`;
      const remove = document.createElement('button'); remove.textContent = 'Remove'; remove.setAttribute('aria-label',`Remove ${zone.name}`);
      remove.disabled = !connected || mapBusy || !!pending || active();
      remove.onclick = () => save({ ...config, zones: config.zones.filter(z => z.id !== zone.id) });
      row.append(text,remove); $('zoneList').append(row);
      const option = document.createElement('option'); option.value = zone.id; option.textContent = zone.name; $('taskZone').append(option);
    }
    if (config.zones.some(z => z.id === selected)) $('taskZone').value = selected;
    $('zoneEmpty').hidden = config.zones.length > 0;
    const base = config.base;
    $('baseStatus').textContent = base ? `Base: (${base.x.toFixed(2)}, ${base.y.toFixed(2)}) m · ${base.yaw_deg.toFixed(0)}°` : 'Choose a return position before starting deliveries.';
    $('taskList').replaceChildren();
    tasks.forEach((task,index) => {
      const row = document.createElement('li'); row.className = 'feature-item';
      const text = document.createElement('span'); text.textContent = `${config.zones.find(z => z.id === task.zone_id)?.name ?? 'Removed zone'} · wait ${task.delay_s}s`;
      const remove = document.createElement('button'); remove.textContent = 'Remove'; remove.setAttribute('aria-label',`Remove task ${index+1}`);
      remove.disabled = active(); remove.onclick = () => { tasks.splice(index,1); render(); };
      row.append(text,remove); $('taskList').append(row);
    });
    const locked = !connected || !configReady || mapBusy || !!pending || active();
    for (const id of ['zoneName','zoneRadius','placeZone','pickBase','currentBase','taskZone','taskDelay','addTask','returnAfter']) $(id).disabled = locked;
    $('startDelivery').disabled = locked || !base || !tasks.length || tasks.some(t => !config.zones.some(z => z.id === t.zone_id));
    $('returnBase').disabled = locked || !base;
    $('cancelDelivery').disabled = !connected || !active();
    $('addTask').disabled ||= !config.zones.length;
  }
  function save(value) {
    try { value = validateNavigation(value); } catch (e) { status(e.message); return; }
    if (!connected || mapBusy || pending || active()) return;
    releaseDrive();
    const request_id = uuid();
    if (!send({ type:'set_navigation_config', ...value, request_id })) { status('Reconnect to save the zones.'); return; }
    pending = request_id; status('Saving positions on the robot…'); render();
    clearTimeout(timer); timer = setTimeout(() => { pending = null; status('No confirmation received. Reconnect and retry.'); render(); },5000);
  }
  function begin(value) {
    placement = value; pickView(); $('placementHint').hidden = false;
    $('placementText').textContent = value.type === 'zone' ? `Click the map to place “${value.name}” (radius ${value.radius} m).` : 'Click the map to place the delivery base (heading 0°).';
    $('mapCanvas').style.cursor = 'crosshair'; releaseDrive();
  }
  $('placeZone').onclick = () => {
    try {
      const name = $('zoneName').value.trim(), radius = Number($('zoneRadius').value);
      validateNavigation({ ...config, zones:[...config.zones,{id:uuid(),name,radius,x:0,y:0}] });
      begin({type:'zone',name,radius}); status('Select the circle centre on the 2D map.');
    } catch (e) { status(e.message); }
  };
  $('pickBase').onclick = () => begin({type:'base'});
  $('currentBase').onclick = () => { const p=robot(); save({ ...config, base:{ x:p.x,y:p.y,yaw_deg:p.yaw*180/Math.PI } }); };
  $('cancelPlacement').onclick = cancelPlacement;
  window.addEventListener('keydown',e => { if (e.key === 'Escape') cancelPlacement(); });
  $('addTask').onclick = () => {
    const delay_s=Number($('taskDelay').value), zone_id=$('taskZone').value;
    if (!zone_id || !Number.isFinite(delay_s) || delay_s<0 || delay_s>86400 || tasks.length>=128) { $('deliveryStatus').textContent='Choose a destination and a delay between 0 and 86400 seconds (maximum 128 tasks).'; return; }
    tasks.push({ zone_id,delay_s }); render();
  };
  $('startDelivery').onclick = () => { releaseDrive(); send({type:'delivery_start',tasks,return_to_base:$('returnAfter').checked,request_id:uuid()}); };
  $('returnBase').onclick = () => { releaseDrive(); send({type:'delivery_return',request_id:uuid()}); };
  $('cancelDelivery').onclick = () => { releaseDrive(); send({type:'nav_stop'}); };
  render();
  return {
    getConfig: () => structuredClone(config),
    setBusy(value) { mapBusy = value; if(value) cancelPlacement(); render(); },
    connection(value) { connected=value; if(!value) { cancelPlacement(); clearTimeout(timer); pending=null; configReady=false; status('Robot disconnected. Reconnect to edit zones.'); } else status('Connected. Waiting for zone configuration…'); render(); },
    pick(x,y) {
      if (!placement) return false;
      const selection = placement; cancelPlacement();
      if (selection.type === 'zone') save({...config,zones:[...config.zones,{id:uuid(),name:selection.name,radius:selection.radius,x,y}]});
      else save({...config,base:{x,y,yaw_deg:0}});
      return true;
    },
    receive(message) {
      if (message.type === 'command_result' && message.request_id === pending) { clearTimeout(timer); pending=null; status(message.reason || 'Could not save.'); }
      const incoming=message.navigation_config;
      if (incoming) {
        try { config=validateNavigation(incoming); configReady=true; } catch (e) { log(`Invalid zone configuration: ${e.message}`); return; }
        if (incoming.request_id === pending && incoming.state !== 'pending') { clearTimeout(timer); pending=null; status(incoming.state === 'error' ? incoming.reason : 'Zones and base saved on the robot.'); }
      }
      if(message.navigation?.delivery) delivery=message.navigation.delivery;
      render();
      if(message.navigation?.state === 'invalid_request') $('deliveryStatus').textContent = message.navigation.reason || 'Delivery request rejected.';
    }
  };
}
