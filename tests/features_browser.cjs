// Real browser -> WS -> production ROS adapters -> TCP -> controller test devices.
const {chromium}=require('../.test_runs/ui/node_modules/playwright');
const assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const path=require('node:path'),fs=require('node:fs');
const root=path.resolve(__dirname,'..');

(async()=>{
  const child=spawn(process.env.PYTHON || 'python',['-u','tests/serve_test_stack.py'],{cwd:root,stdio:['pipe','pipe','pipe']});
  let output='',errors='',browser;
  child.stderr.on('data',data=>errors+=data);
  const endpoint=await new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{child.kill();reject(new Error(errors || 'Stack timed out'));},10000);
    child.once('exit',code=>{clearTimeout(timer);reject(new Error(`Stack exited ${code}: ${errors}`));});
    child.stdout.on('data',data=>{output+=data;const match=output.match(/READY (\{[^\n]+\})/);if(match){clearTimeout(timer);resolve(JSON.parse(match[1]));}});
  });
  try {
    browser=await chromium.launch({channel:'chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const context=await browser.newContext({viewport:{width:1280,height:900},acceptDownloads:true});
    await context.addInitScript(port=>{
      const Native=window.WebSocket;
      window.testCommands=[];window.testPacketCount=0;
      window.WebSocket=class extends Native {
        constructor(url,...args){const target=new URL(url);target.port=port;super(target.toString(),...args);this.addEventListener('message',e=>{window.testPacketCount++;window.lastTestPacket=JSON.parse(e.data);});}
        send(raw){window.testCommands.push(JSON.parse(raw));super.send(raw);}
      };
    },String(endpoint.ws_port));
    const page=await context.newPage(),pageErrors=[];
    page.on('pageerror',e=>pageErrors.push(e.message));
    const base=`http://127.0.0.1:${endpoint.http_port}`;
    const state=async()=> (await context.request.get(base+'/__test_state')).json();
    const until=async(predicate,timeout=6000)=>{
      const end=Date.now()+timeout;
      while(!await predicate()) {
        if(Date.now()>end)throw new Error('Feature condition timed out: '+JSON.stringify({state:await state(),delivery:await page.locator('#deliveryStatus').textContent(),zones:await page.locator('#zoneStatus').textContent(),operation:await page.locator('#operationStatus').textContent(),predicate:String(predicate),pageErrors,browser:await page.evaluate(()=>({status:document.querySelector('#status').textContent,commands:window.testCommands.slice(-3).map(({grid,...c})=>c),config:window.lastTestPacket?.navigation_config,logs:document.querySelector('#log').textContent}))}));
        await page.waitForTimeout(40);
      }
    };
    async function importFile(file) {
      await page.locator('#knownMapFile').setInputFiles(file);await page.locator('#loadKnownMap').click();
      await page.waitForFunction(()=>document.querySelector('#sendGoal').disabled);
      await until(async()=> !(await page.locator('#sendGoal').isDisabled()),31000);
      assert.match(await page.locator('#operationStatus').textContent(),/localized and ready/);
    }
    async function pick(x,y) {
      const location=await page.evaluate(async({x,y})=>{
        const {GridRenderer}=await import('/js/grid-renderer.js?v=delivery-v8');
        const canvas=document.querySelector('#mapCanvas'),rect=canvas.getBoundingClientRect(),v=GridRenderer.getView();
        const scale=Math.min(rect.width,rect.height)/(2*Number(document.querySelector('#radius').value))*v.zoom;
        return {x:rect.x+rect.width/2+(x-v.panX)*scale,y:rect.y+rect.height/2-(y-v.panY)*scale};
      },{x,y});
      await page.mouse.click(location.x,location.y);
      await until(async()=> /saved on the robot/.test(await page.locator('#zoneStatus').textContent()));
    }
    async function addZone(name,x,y) {
      await page.locator('#zoneName').fill(name);await page.locator('#zoneRadius').fill('.1');
      await page.locator('#placeZone').click();assert.equal(await page.locator('#placementHint').isVisible(),true);
      await pick(x,y);
      assert.equal(await page.locator('#arrivalDialog').isVisible(),false);
    }
    await page.goto(base+'/map_viewer2_modular.html');
    await until(async()=>await page.locator('#placeZone').isEnabled());
    const room=await (await context.request.get(base+'/__test_map.png')).body();
    await page.locator('#knownMapResolution').fill('.1');await page.locator('#knownMapRadius').fill('6');
    await importFile({name:'room.png',mimeType:'image/png',buffer:room});
    console.log('Chrome feature map import confirmed');
    await page.locator('#placeZone').click();assert.match(await page.locator('#zoneStatus').textContent(),/unique name/);
    await page.locator('#zoneName').fill('Station A');await page.locator('#zoneRadius').fill('-1');await page.locator('#placeZone').click();
    assert.match(await page.locator('#zoneStatus').textContent(),/radius/);
    await page.locator('#zoneRadius').fill('.1');await page.locator('#placeZone').click();await page.locator('#cancelPlacement').click();
    assert.equal(await page.locator('#placementHint').isVisible(),false);
    assert.equal(await page.locator('#zoneList .feature-item').count(),0);
    await page.locator('#currentBase').click();console.log('Chrome base request sent');await until(async()=> /Base:/.test(await page.locator('#baseStatus').textContent()));
    await addZone('Station A',1.15,.35);await addZone('Station B',.15,-1.1);
    assert.equal(await page.locator('#zoneList .feature-item').count(),2);
    await page.locator('#placeZone').click();assert.match(await page.locator('#zoneStatus').textContent(),/unique name/);
    await page.locator('#taskZone').selectOption({label:'Station A'});await page.locator('#taskDelay').fill('.2');await page.locator('#addTask').click();
    await page.locator('#taskZone').selectOption({label:'Station B'});await page.locator('#taskDelay').fill('.3');await page.locator('#addTask').click();
    assert.equal(await page.locator('#taskList li').count(),2);assert.equal(await page.locator('#returnAfter').isChecked(),true);
    await page.locator('#startDelivery').click();
    await until(async()=> /Approaching|Waiting/.test(await page.locator('#deliveryStatus').textContent()));
    assert.equal(await page.locator('#placeZone').isDisabled(),true);
    await until(async()=> /Delivery complete/.test(await page.locator('#deliveryStatus').textContent()),65000);
    assert.match(await page.locator('#deliveryStatus').textContent(),/2\/2 tasks done/);
    const returned=await state(),savedBase=returned.config.base;
    assert(Math.hypot(returned.pose[0]-savedBase.x,returned.pose[1]-savedBase.y)<.06);
    const jsonDownload=page.waitForEvent('download');await page.locator('#saveBundle').click();
    const jsonFile=await (await jsonDownload).path(),json=fs.readFileSync(jsonFile);
    const bundle=JSON.parse(json);assert.equal(bundle.navigation_config.zones.length,2);assert(bundle.navigation_config.base);
    const pngDownload=page.waitForEvent('download');await page.locator('#save').click();
    const pngFile=await (await pngDownload).path(),png=fs.readFileSync(pngFile);
    const meta=await page.evaluate(async bytes=>{
      const {readPNGMetadata}=await import('/js/map-exporter.js?v=delivery-v8');return readPNGMetadata(new Uint8Array(bytes));
    },Array.from(png));
    assert.deepEqual(meta.navigation_config,bundle.navigation_config);
    await page.getByRole('button',{name:'Remove Station A',exact:true}).click();await until(async()=>await page.locator('#zoneList .feature-item').count()===1);
    await importFile({name:'saved-map.json',mimeType:'application/json',buffer:json});
    await until(async()=>await page.locator('#zoneList .feature-item').count()===2);
    await page.getByRole('button',{name:'Remove Station B',exact:true}).click();await until(async()=>await page.locator('#zoneList .feature-item').count()===1);
    await importFile({name:'saved-map.png',mimeType:'image/png',buffer:png});
    await until(async()=>await page.locator('#zoneList .feature-item').count()===2);
    assert.deepEqual((await state()).errors,[]);assert.deepEqual(pageErrors,[]);
    const results=path.join(root,'tests/results');fs.mkdirSync(results,{recursive:true});
    await page.locator('#zoneName').evaluate(input=>input.closest('section').scrollIntoView({block:'start'}));await page.screenshot({path:path.join(results,'features-desktop.png')});
    await page.setViewportSize({width:390,height:844});await page.locator('#zoneName').scrollIntoViewIfNeeded();
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    await page.screenshot({path:path.join(results,'features-mobile.png'),fullPage:true});
    fs.writeFileSync(path.join(results,'features-browser-validation.json'),JSON.stringify({result:'passed',scope:'Chrome and production HTML/JS, real WS/TCP, production adapters and controller; ROS topics and Webots devices simulated; localization defaults enabled',
      checks:['circle placement and form validation','base selection','ordered delivery with delays and return','PNG/JSON zone and base round trips','desktop/mobile fit','no browser or adapter errors']},null,2)+'\n');
    console.log('Chrome features: circle placement/validation, base, two-stop delivery + waits/return, PNG and JSON restoration, desktop/mobile layout — PASS');
  } finally {
    if(browser)await browser.close();child.stdin.write('stop\n');
    await new Promise(resolve=>{if(child.exitCode!==null)return resolve();const timer=setTimeout(()=>{child.kill();resolve();},5000);child.once('exit',()=>{clearTimeout(timer);resolve();});});
  }
})().catch(e=>{console.error(e);process.exitCode=1;});
