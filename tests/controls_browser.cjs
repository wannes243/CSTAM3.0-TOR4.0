// Real production HTML, 2D/3D renderers and input events in Chrome.
// Only the unavailable ROS WebSocket is replaced; no external network is used.
const { chromium } = require('../.test_runs/ui/node_modules/playwright');
const assert = require('node:assert/strict');
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
const output = path.join(root, '.test_runs/controls_v5');
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
    browser = await chromium.launch({channel: 'chrome', headless: true,
      args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
    const context = await browser.newContext({viewport: {width: 1280, height: 900}, hasTouch: true});
    await context.route('https://**', route => route.abort());
    await context.addInitScript(() => {
      window.sentCommands = [];
      window.WebSocket = class extends EventTarget {
        static OPEN = 1;
        constructor() {
          super(); this.readyState = 1; window.robotSocket = this;
          setTimeout(() => this.dispatchEvent(new Event('open')), 0);
        }
        send(payload) {
          const command=JSON.parse(payload);
          window.sentCommands.push(command);
          if (['clear_map','reset_odom','configure_map','nav_clear_map'].includes(command.type)) {
            setTimeout(() => this.dispatchEvent(new MessageEvent('message', {data:JSON.stringify({
              type:'scan',operation:{request_id:command.request_id,state:'complete'},navigation:{},safety:{}
            })})),50);
          }
        }
        close() { this.readyState = 3; this.dispatchEvent(new Event('close')); }
      };
    });
    const page = await context.newPage();
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    const url = `http://127.0.0.1:${server.address().port}/map_viewer2_modular.html`;
    await page.goto(url);
    await page.waitForFunction(() => document.getElementById('status').textContent === 'Connected');
    assert.equal(await page.locator('#knownMapRadius').evaluate(input=>{
      const field=input.getBoundingClientRect(),section=input.closest('section').getBoundingClientRect();
      return field.right<=section.right;
    }),true,'map scale fields stay inside the sidebar');
    const last = () => page.evaluate(() => window.sentCommands.at(-1));

    for (const [direction, sign] of [['left', 1], ['right', -1]]) {
      const button = page.locator(`[data-d="${direction}"]`);
      const box = await button.boundingBox();
      const before = await page.evaluate(() => window.sentCommands.length);
      await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
      await page.mouse.down();
      await page.waitForTimeout(350);
      assert.equal((await last()).v, 0);
      assert.ok((await last()).omega * sign > 0);
      assert.ok(await page.evaluate(count => window.sentCommands.length >= count + 3, before));
      assert.ok(await button.evaluate(el => el.classList.contains('active')));
      // Pointer capture keeps turning when the pointer leaves the button.
      await page.mouse.move(700, 120);
      await page.waitForTimeout(120);
      assert.ok((await last()).omega * sign > 0);
      await page.mouse.up();
      assert.deepEqual(await last(), {type: 'drive', v: 0, omega: 0});
    }
    for (const [key, sign] of [['ArrowLeft', 1], ['ArrowRight', -1]]) {
      await page.keyboard.down(key);
      await page.waitForTimeout(150);
      assert.ok((await last()).omega * sign > 0);
      await page.keyboard.up(key);
      assert.equal((await last()).omega, 0);
    }
    await page.locator('[data-d="left"]').focus();
    const beforeSpace = await page.evaluate(() => window.sentCommands.length);
    await page.keyboard.down('Space');
    assert.ok((await last()).omega > 0);
    await page.keyboard.up('Space');
    assert.equal((await last()).omega, 0);
    assert.equal(await page.evaluate(count => window.sentCommands.slice(count).some(m => m.type === 'toggle_scan'), beforeSpace), false);

    await page.evaluate(() => window.robotSocket.dispatchEvent(new MessageEvent('message', {
      data: JSON.stringify({type: 'scan', robot: {x: 2, y: 1, theta: .5},
        new_points: [[2, 2, .3]], points_frame: 'map',
        safety: {stop_active: true, reason: 'rotation_obstacle=0.48m'}, navigation: {state: 'idle'}})
    })));
    assert.match(await page.locator('#safetyStatus').textContent(), /Rotation bloquée/);
    await page.evaluate(() => window.robotSocket.dispatchEvent(new MessageEvent('message', {
      data: JSON.stringify({type: 'scan', robot: {x: 2, y: 1, theta: .5},
        safety: {stop_active: true, reason: 'rear_obstacle=0.44m'}})
    })));
    assert.match(await page.locator('#safetyStatus').textContent(), /Recul bloqué/);
    assert.match(await page.locator('#safetyStatus').textContent(), /Avancez/);
    const canvas = page.locator('#mapCanvas');
    const imageBefore = await canvas.evaluate(el => el.toDataURL());
    await canvas.hover();
    await page.mouse.wheel(0, -300);
    await page.waitForTimeout(80);
    assert.notEqual(await canvas.evaluate(el => el.toDataURL()), imageBefore, 'zoom redraws without new robot data');
    const imageZoomed = await canvas.evaluate(el => el.toDataURL());
    const box = await canvas.boundingBox();
    await page.mouse.move(box.x + 100, box.y + 100);
    await page.mouse.down();
    await page.mouse.move(box.x + 140, box.y + 130);
    await page.mouse.up();
    assert.notEqual(await canvas.evaluate(el => el.toDataURL()), imageZoomed, 'pan redraws without new robot data');
    assert.equal(await page.locator('#arrivalDialog').isVisible(), false);

    await page.route('**/vendor/three/three.module.js', async route => {
      await new Promise(resolve => setTimeout(resolve, 400));
      await route.continue();
    });
    await page.locator('#mode3d').click();
    await page.locator('#mode2d').click();
    await page.waitForTimeout(1000);
    assert.equal(await page.locator('#view2d').isVisible(), true, 'late 3D loading respects the selected 2D view');
    await page.locator('#mode3d').click();
    await page.waitForFunction(() => document.getElementById('view3d').classList.contains('active'));
    await page.waitForTimeout(200);
    assert.equal(await page.locator('#mode3d').getAttribute('aria-pressed'), 'true');
    await page.screenshot({path: path.join(output, 'desktop-3d.png')});
    await page.locator('#mode2d').click();
    assert.equal(await canvas.isVisible(), true);
    await page.locator('#resetMap').click();
    assert.equal((await last()).type, 'clear_map');
    await page.locator('#explore').click();
    assert.equal((await last()).type, 'nav_explore');
    await page.locator('#stop').click();
    assert.equal((await last()).type, 'stop');

    await page.setViewportSize({width: 390, height: 844});
    await page.locator('#mode3d').click();
    assert.equal(await page.locator('#view3d').isVisible(), true);
    await page.locator('#mode2d').click();
    const mobileButton = page.locator('[data-d="right"]');
    await mobileButton.scrollIntoViewIfNeeded();
    const touchBox = await mobileButton.boundingBox();
    const cdp = await context.newCDPSession(page);
    const touch = {x: touchBox.x + touchBox.width / 2, y: touchBox.y + touchBox.height / 2};
    await cdp.send('Input.dispatchTouchEvent', {type: 'touchStart', touchPoints: [touch]});
    await page.waitForTimeout(250);
    assert.ok((await last()).omega < 0);
    await cdp.send('Input.dispatchTouchEvent', {type: 'touchMove', touchPoints: [{x:touch.x + 4, y:touch.y + 12}]});
    await page.waitForTimeout(150);
    assert.ok((await last()).omega < 0, 'touch movement does not cancel the turn');
    await cdp.send('Input.dispatchTouchEvent', {type: 'touchEnd', touchPoints: []});
    assert.equal((await last()).omega, 0);
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), true);
    await page.screenshot({path: path.join(output, 'mobile-controls.png')});

    // A missing renderer stays recoverable: 2D and drive controls still work.
    const offline = await context.newPage();
    offline.on('pageerror', error => errors.push(error.message));
    await offline.route('**/vendor/three/**', route => route.abort());
    await offline.goto(url);
    await offline.locator('#mode3d').click();
    await offline.waitForFunction(() => document.getElementById('viewStatus').textContent.includes('indisponible'));
    assert.equal(await offline.locator('#view2d').isVisible(), true);
    await offline.keyboard.press('ArrowLeft');
    assert.equal(await offline.evaluate(() => window.sentCommands.at(-1).omega), 0);
    assert.ok(await offline.evaluate(() => window.sentCommands.some(m => m.omega > 0)));

    const noWebGL = await context.newPage();
    noWebGL.on('pageerror', error => errors.push(error.message));
    await noWebGL.addInitScript(() => {
      const getContext = HTMLCanvasElement.prototype.getContext;
      HTMLCanvasElement.prototype.getContext = function(type, ...args) {
        return /webgl/i.test(type) ? null : getContext.call(this, type, ...args);
      };
    });
    await noWebGL.goto(url);
    await noWebGL.locator('#mode3d').click();
    await noWebGL.waitForFunction(() => document.getElementById('viewStatus').textContent.includes('indisponible'));
    assert.equal(await noWebGL.locator('#view2d').isVisible(), true);
    await noWebGL.locator('#mode2d').click();
    assert.equal(await noWebGL.locator('#viewStatus').textContent(), '');

    // The PNG importer preserves Y direction, unknown gray and transparency.
    const encoded=await page.evaluate(()=>{
      const canvas=document.createElement('canvas');canvas.width=canvas.height=2;
      const ctx=canvas.getContext('2d');
      ctx.fillStyle='#000';ctx.fillRect(0,0,1,1);
      ctx.fillStyle='#fff';ctx.fillRect(1,0,1,1);
      ctx.fillStyle='#808080';ctx.fillRect(0,1,1,1);
      return canvas.toDataURL('image/png').split(',')[1];
    });
    await page.locator('#knownMapFile').setInputFiles({name:'cells.png',mimeType:'image/png',buffer:Buffer.from(encoded,'base64')});
    await page.locator('#knownMapResolution').fill('1');await page.locator('#knownMapRadius').fill('1');
    await page.locator('#loadKnownMap').click();
    await page.waitForFunction(()=>window.sentCommands.at(-1)?.type==='nav_map');
    const imported=await last();
    assert.deepEqual(imported.grid,[-1,-1,100,0]);
    assert.equal(await page.locator('#sendGoal').isDisabled(),true);
    await page.evaluate(request_id=>window.robotSocket.dispatchEvent(new MessageEvent('message',{
      data:JSON.stringify({type:'command_result',request_id,state:'error',reason:'Image cannot be localized'})
    })),imported.request_id);
    assert.equal(await page.locator('#sendGoal').isDisabled(),false);
    assert.match(await page.locator('#operationStatus').textContent(),/cannot be localized/);
    assert.deepEqual(errors, []);
    console.log('Chrome: sustained left/right, release, keyboard, touch, immediate zoom/pan, real offline 3D, view switching, front/rear safety status and map clearing — PASS');
  } finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
  }
})().catch(error => {console.error(error); process.exitCode = 1;});
