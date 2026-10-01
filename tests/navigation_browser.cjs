// Real Chromium DOM/keyboard checks. ROS WebSocket and the unused 3D renderer
// are substituted; the 2D renderer, app and HTML are the production files.
const { chromium } = require('../.test_runs/ui/node_modules/playwright');
const assert = require('node:assert/strict');
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
const output = path.join(root, '.test_runs/navigation_v4');
fs.mkdirSync(output, { recursive: true });

(async () => {
  const server = http.createServer((req, res) => {
    const file = path.resolve(root, '.' + new URL(req.url, 'http://localhost').pathname);
    if (!file.startsWith(root + path.sep)) { res.writeHead(403).end(); return; }
    fs.readFile(file, (error, content) => {
      if (error) { res.writeHead(404).end(); return; }
      res.setHeader('Content-Type', file.endsWith('.js') ? 'text/javascript' : 'text/html');
      res.end(content);
    });
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({channel: 'chrome', headless: true});
    const page = await browser.newPage({viewport: {width: 1280, height: 900}});
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.route('**/js/three-renderer.js*', route => route.fulfill({
      contentType: 'text/javascript', body: 'export const ThreeDRenderer = new Proxy({}, {get:()=>()=>{}});'
    }));
    await page.addInitScript(() => {
      window.sentCommands = [];
      window.WebSocket = class extends EventTarget {
        static OPEN = 1;
        constructor() { super(); this.readyState = 1; setTimeout(() => this.dispatchEvent(new Event('open')), 0); }
        send(payload) { window.sentCommands.push(JSON.parse(payload)); }
      };
    });
    await page.goto(`http://127.0.0.1:${server.address().port}/map_viewer2_modular.html`);
    await page.waitForFunction(() => document.getElementById('status').textContent === 'Connected');
    await page.locator('#mapCanvas').click({position: {x: 600, y: 250}});
    assert.equal(await page.locator('#arrivalDialog').isVisible(), true);
    assert.equal(await page.evaluate(() => window.sentCommands.length), 0);
    assert.equal(await page.locator('#arrivalYaw').evaluate(el => el === document.activeElement), true);
    await page.keyboard.press('ArrowLeft');
    assert.equal(await page.evaluate(() => window.sentCommands.length), 0);
    await page.getByRole('button', {name: 'Démarrer'}).click();
    assert.match(await page.locator('#arrivalError').textContent(), /Saisissez/);
    await page.locator('#arrivalYaw').fill('80');
    await page.screenshot({path: path.join(output, 'angle-desktop.png')});
    await page.getByRole('button', {name: 'Démarrer'}).click();
    assert.equal(await page.locator('#arrivalDialog').isVisible(), false);
    assert.equal(await page.evaluate(() => window.sentCommands.at(-1).yaw_deg), 80);
    await page.locator('#mapCanvas').click({position: {x: 550, y: 300}});
    await page.keyboard.press('Escape');
    assert.equal(await page.locator('#arrivalDialog').isVisible(), false);
    assert.equal(await page.evaluate(() => window.sentCommands.length), 1);
    await page.locator('#mapCanvas').click({position: {x: 550, y: 300}});
    await page.setViewportSize({width: 390, height: 844});
    const box = await page.locator('#arrivalDialog').boundingBox();
    assert.ok(box.x >= 0 && box.x + box.width <= 390 && box.y >= 0 && box.y + box.height <= 844);
    await page.locator('#arrivalWithoutAngle').check();
    assert.equal(await page.locator('#arrivalYaw').isDisabled(), true);
    await page.screenshot({path: path.join(output, 'angle-mobile.png')});
    await page.getByRole('button', {name: 'Démarrer'}).click();
    assert.equal(await page.evaluate(() => 'yaw_deg' in window.sentCommands.at(-1)), false);
    assert.deepEqual(errors, []);
    console.log('Chrome: map click, focus, blocked driving, validation, 80°, Escape, no angle, mobile fit — PASS');
  } finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
  }
})().catch(error => {console.error(error); process.exitCode = 1;});
