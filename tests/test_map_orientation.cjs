const assert = require('node:assert/strict');
const { test } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

test('turning the robot keeps map cells and wall points fixed on screen', () => {
  const rectangles = [], rotations = [];
  const context2d = {
    setTransform() {}, fillRect: (...args) => rectangles.push(args),
    save() {}, restore() {}, translate() {}, rotate: angle => rotations.push(angle),
    beginPath() {}, moveTo() {}, lineTo() {}, closePath() {}, fill() {}, stroke() {}
  };
  const canvas = { clientWidth: 400, clientHeight: 400, width: 400, height: 400,
    getContext: () => context2d, addEventListener() {} };
  const context = vm.createContext({ devicePixelRatio: 1, canvas });
  const source = fs.readFileSync(path.join(__dirname, '../js/grid-renderer.js'), 'utf8');
  vm.runInContext(source.replace('export const', 'const'), context);
  vm.runInContext('GridRenderer.init(canvas)', context);
  context.map = { size: 2, radius: 5, resolution: 5, grid: [0, 100, -1, 100] };
  let firstMap;
  for (const yaw of [0, Math.PI/2, Math.PI, -Math.PI/2]) {
    context.robot = { x: 1, y: 0, yaw };
    rectangles.length = 0;
    vm.runInContext('GridRenderer.render(map, robot, [[2, 1]])', context);
    const actual = JSON.stringify(rectangles);
    firstMap ??= actual;
    assert.equal(actual, firstMap);
    // A +90 degree heading points up on a canvas whose +Y points down.
    const angle = rotations.at(-1);
    assert.ok(Math.abs(Math.cos(angle)-Math.cos(yaw)) < 1e-12);
    assert.ok(Math.abs(Math.sin(angle)+Math.sin(yaw)) < 1e-12);
  }
});
