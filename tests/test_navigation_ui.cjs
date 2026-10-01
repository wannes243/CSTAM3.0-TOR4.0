const assert = require('node:assert/strict');
const { test } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function viewer() {
  const elements = new Map();
  const messages = [], logs = [], listeners = {}, intervals = [], socketListeners = {}, timeouts = [];
  let clears = 0, resets = 0;
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value: '', validity: { valid: true }, reportValidity() {},
      classList: { add() {}, remove() {}, toggle() {} }, setAttribute() {},
      open: false, checked: false, focus() {},
      events: {}, addEventListener(name, fn) { this.events[name] = fn; },
      showModal() { this.open = true; },
      close() { this.open = false; this.events.close?.(); }
    });
    return elements.get(id);
  };
  let connected = true, click;
  const noop = () => {};
  const context = vm.createContext({
    createNavigationFeatures: () => ({setBusy:noop,connection:noop,receive:noop,pick:()=>false,getConfig:()=>({zones:[],base:null})}),
    UIController: { $: element, log: text => logs.push(text), updateDrive: noop, updateSafety: noop, setStatus: noop, updateCounters: noop, updateRobot: noop },
    WebSocketManager: { send: message => { if (connected) messages.push(JSON.parse(JSON.stringify(message))); return connected; }, on: (type, handler) => { socketListeners[type] = handler; }, connect: noop },
    MapModel: { allocate: noop, clear: () => clears++, get: () => ({}) },
    RobotState: { get: () => ({}), reset: () => resets++ },
    GridRenderer: { init: noop, setClickHandler: handler => { click = handler; }, render: noop, resize: noop },
    ThreeDRenderer: { init: noop, setVisible: noop, animate: noop, clearPoints: noop, updateRobot: noop },
    document: { querySelectorAll: () => [] },
    window: { addEventListener: (type, handler) => { listeners[type] = handler; } },
    setInterval: handler => intervals.push(handler),
    setTimeout: handler => { timeouts.push(handler); return timeouts.length; }, clearTimeout: noop
  });
  const source = fs.readFileSync(path.join(__dirname, '../js/app.js'), 'utf8').replace(/^import .*;\r?\n/gm, '');
  vm.runInContext(source, context);
  return { element, messages, logs, listeners, click: (...args) => click(...args),
    submit: () => element('arrivalForm').onsubmit({preventDefault(){}}),
    intervals, socketListeners, timeouts, clears: () => clears, resets: () => resets,
    disconnect: () => { connected = false; } };
}

test('coordinate button transmits final orientation, including zero', () => {
  const ui = viewer();
  ui.element('goalX').value = '10';
  ui.element('goalY').value = '0';
  for (const degrees of ['0', '80', '90', '180']) {
    ui.element('goalYaw').value = degrees;
    ui.element('sendGoal').onclick();
    assert.deepEqual(ui.messages.at(-1), { type: 'nav_goal', x: 10, y: 0, yaw_deg: Number(degrees) });
  }
});

test('map click asks for the angle and sends nothing until confirmed', () => {
  const ui = viewer();
  ui.element('goalYaw').value = '90';
  ui.click(10, 2);
  assert.equal(ui.messages.length, 0);
  assert.equal(ui.element('arrivalDialog').open, true);
  ui.submit();
  assert.equal(ui.messages.at(-1).yaw_deg, 90);
  assert.equal(ui.element('arrivalDialog').open, false);
  ui.element('goalYaw').value = '';
  ui.click(5, 3);
  ui.submit();
  assert.equal(ui.messages.length, 1);
  ui.element('arrivalWithoutAngle').checked = true;
  ui.submit();
  assert.deepEqual(ui.messages.at(-1), { type: 'nav_goal', x: 5, y: 3 });
});

test('blank coordinates and invalid angle do not send a goal', () => {
  const ui = viewer();
  ui.element('sendGoal').onclick();
  assert.equal(ui.messages.length, 0);
  ui.element('goalX').value = '10';
  ui.element('goalY').value = '0';
  ui.element('goalYaw').validity.valid = false;
  ui.element('sendGoal').onclick();
  assert.equal(ui.messages.length, 0);
});

test('disconnected controls report failure instead of confirming movement or stop', () => {
  const ui = viewer();
  ui.disconnect();
  ui.click(10, 0);
  ui.element('arrivalYaw').value = '80';
  ui.submit();
  assert.equal(ui.element('arrivalDialog').open, true);
  assert.match(ui.logs.at(-1), /not sent/);
  ui.element('navStop').onclick();
  assert.match(ui.logs.at(-1), /not sent/);
  assert.equal(ui.messages.length, 0);
});

test('Stop Nav transmits cancellation and waits for acknowledgement', () => {
  const ui = viewer();
  ui.element('navStop').onclick();
  assert.deepEqual(ui.messages.at(-1), { type: 'nav_stop' });
  assert.match(ui.logs.at(-1), /requested/);
});

test('arrow keys in a goal input do not drive the robot', () => {
  const ui = viewer();
  ui.listeners.keydown({ key: 'ArrowUp', target: { closest: () => ({}) } });
  assert.equal(ui.messages.length, 0);
  ui.listeners.keydown({ key: 'ArrowUp', target: { closest: () => null }, preventDefault() {} });
  assert.equal(ui.messages.at(-1).type, 'drive');
  assert.ok(ui.messages.at(-1).v > 0);
});

test('losing window focus releases a held turn', () => {
  const ui = viewer();
  ui.listeners.keydown({ key: 'ArrowLeft', target: { closest: () => null }, preventDefault() {} });
  assert.ok(ui.messages.at(-1).omega > 0);
  ui.listeners.blur();
  assert.deepEqual(ui.messages.at(-1), { type: 'drive', v: 0, omega: 0 });
});

test('selecting a goal releases manual rotation before sending the goal', () => {
  const ui = viewer();
  ui.listeners.keydown({ key: 'ArrowLeft', target: { closest: () => null }, preventDefault() {} });
  ui.click(1, 0);
  assert.deepEqual(ui.messages.at(-1), { type: 'drive', v: 0, omega: 0 });
  ui.element('arrivalYaw').value = '0';
  ui.submit();
  assert.equal(ui.messages.at(-1).type, 'nav_goal');
});

test('cancel discards the pending point and modal keys never drive', () => {
  const ui = viewer();
  ui.click(1, 2);
  ui.listeners.keydown({key:'ArrowLeft',target:{closest:()=>null},preventDefault(){}});
  assert.equal(ui.messages.length, 0);
  ui.element('arrivalCancel').onclick();
  ui.submit();
  assert.equal(ui.messages.length, 0);
  ui.click(3, 4);
  ui.element('arrivalYaw').value = '180';
  ui.submit();
  assert.deepEqual(ui.messages.at(-1), {type:'nav_goal',x:3,y:4,yaw_deg:180});
});

test('invalid modal angle keeps confirmation open without sending', () => {
  const ui = viewer();
  ui.click(1, 2);
  ui.element('arrivalYaw').value = 'Infinity';
  ui.submit();
  assert.equal(ui.messages.length, 0);
  assert.equal(ui.element('arrivalDialog').open, true);
});

test('both turn directions refresh while held and stop on release', () => {
  for (const [key, sign] of [['ArrowLeft', 1], ['ArrowRight', -1]]) {
    const ui = viewer();
    const event = {key, target: {closest: () => null}, preventDefault(){}};
    ui.listeners.keydown(event);
    for (let i = 0; i < 5; i++) ui.intervals[0]();
    assert.equal(ui.messages.length, 6);
    assert.ok(ui.messages.every(message => message.v === 0 && message.omega * sign > 0));
    ui.listeners.keyup(event);
    assert.deepEqual(ui.messages.at(-1), {type: 'drive', v: 0, omega: 0});
    ui.intervals[0]();
    assert.equal(ui.messages.length, 7);
  }
});

test('releasing one keyboard binding preserves another held binding', () => {
  const ui = viewer();
  const event = key => ({key, target: {closest: () => null}, preventDefault(){}});
  ui.listeners.keydown(event('ArrowLeft'));
  ui.listeners.keydown(event('a'));
  ui.listeners.keyup(event('a'));
  assert.ok(ui.messages.at(-1).omega > 0);
  ui.listeners.keyup(event('ArrowLeft'));
  assert.equal(ui.messages.at(-1).omega, 0);
});

test('a disconnected turn is cleared before reconnecting', () => {
  const ui = viewer();
  ui.listeners.keydown({key:'ArrowRight',target:{closest:()=>null},preventDefault(){}});
  ui.disconnect();
  ui.socketListeners.close();
  const count = ui.messages.length;
  ui.intervals[0]();
  assert.equal(ui.messages.length, count);
});

test('clear map waits for backend confirmation and exploration reports connection failure', () => {
  const ui = viewer();
  ui.element('resetMap').onclick();
  assert.equal(ui.messages.at(-1).type, 'clear_map');
  assert.equal(ui.clears(),0);
  assert.equal(ui.element('sendGoal').disabled,true);
  const request_id=ui.messages.at(-1).request_id;
  ui.socketListeners.message({type:'command_result',request_id,state:'complete'});
  assert.equal(ui.clears(),1);
  assert.equal(ui.element('sendGoal').disabled,false);
  ui.disconnect();
  ui.element('explore').onclick();
  assert.match(ui.logs.at(-1), /non envoyée/);
  assert.equal(ui.messages.length, 1);
});

test('failed reset preserves pose and releases disabled controls', () => {
  const ui=viewer();
  ui.element('resetOdom').onclick();
  const request_id=ui.messages.at(-1).request_id;
  assert.equal(ui.resets(),0);
  ui.socketListeners.message({type:'command_result',request_id:'old',state:'complete'});
  assert.equal(ui.element('sendGoal').disabled,true);
  ui.socketListeners.message({type:'command_result',request_id,state:'error',reason:'No fresh sensors'});
  assert.equal(ui.resets(),0);
  assert.equal(ui.element('sendGoal').disabled,false);
  assert.match(ui.element('operationStatus').textContent,/No fresh sensors/);
});

test('confirmed reset resets position and orientation', () => {
  const ui=viewer();
  ui.element('resetOdom').onclick();
  ui.socketListeners.message({type:'command_result',request_id:ui.messages.at(-1).request_id,state:'complete'});
  assert.equal(ui.resets(),1);
  assert.equal(ui.clears(),1);
});

test('operation timeout permits retry and never clears an unconfirmed map', () => {
  const ui=viewer();
  ui.element('resetMap').onclick();
  ui.timeouts.at(-1)();
  assert.equal(ui.clears(),0);
  assert.equal(ui.element('resetMap').disabled,false);
  assert.match(ui.element('operationStatus').textContent,/did not confirm/);
});
