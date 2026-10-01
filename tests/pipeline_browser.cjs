// Production HTML -> real WebSocket -> adapters -> real TCP -> controller.
// ROS topics and Webots devices are simulated by serve_test_stack.py.
const {chromium}=require('../.test_runs/ui/node_modules/playwright');
const assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const path=require('node:path');
const fs=require('node:fs');
const root=path.resolve(__dirname,'..');

(async()=>{
  const child=spawn(process.env.PYTHON || 'python',['-u','tests/serve_test_stack.py'],{cwd:root,stdio:['pipe','pipe','pipe']});
  let output='',errors='';
  child.stderr.on('data',data=>errors+=data.toString());
  const endpoint=await new Promise((resolve,reject)=>{
      const deadline=setTimeout(()=>{child.kill();reject(new Error('Stack startup timed out: '+errors));},10000);
    child.once('exit',code=>{clearTimeout(deadline);reject(new Error(`Stack exited ${code}: ${errors}`));});
    child.stdout.on('data',data=>{
      output+=data.toString();
      const match=output.match(/READY (\{[^\n]+\})/);
      if(match){clearTimeout(deadline);resolve(JSON.parse(match[1]));}
    });
  });
  let browser;
  try {
    browser=await chromium.launch({channel:'chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const context=await browser.newContext({viewport:{width:1280,height:900}});
    // Preserve native networking; change only the test server's ephemeral port.
    await context.addInitScript(port=>{
      const NativeWebSocket=window.WebSocket;
      window.WebSocket=class extends NativeWebSocket {
        constructor(url,...args){const target=new URL(url);target.port=port;super(target.toString(),...args);}
      };
    },String(endpoint.ws_port));
    const page=await context.newPage();const pageErrors=[];
    page.on('pageerror',error=>pageErrors.push(error.message));
    const base=`http://127.0.0.1:${endpoint.http_port}`;
    const state=async()=> (await context.request.get(base+'/__test_state')).json();
    const until=async(predicate,timeout=5000)=>{
      const deadline=Date.now()+timeout;
      while(!await predicate()){
        if(Date.now()>=deadline)throw new Error('Browser pipeline condition timed out: '+JSON.stringify({state:await state(),
          nav:await page.locator('#navStatus').textContent(),operation:await page.locator('#operationStatus').textContent(),predicate:String(predicate)}));
        await page.waitForTimeout(40);
      }
    };
    await page.goto(base+'/map_viewer2_modular.html');
    await page.waitForFunction(()=>document.querySelector('#status').textContent==='Connected' && Number(document.querySelector('#packets').textContent)>0);
    for(const [direction,sign] of [['left',1],['right',-1]]){
      const box=await page.locator(`[data-d="${direction}"]`).boundingBox();
      await page.mouse.move(box.x+box.width/2,box.y+box.height/2);await page.mouse.down();
      await until(async()=> (await state()).w*sign>.1);
      await page.mouse.up();await until(async()=>Math.abs((await state()).w)<1e-6);
    }
    for(const [direction,sign] of [['up',1],['down',-1]]){
      const box=await page.locator(`[data-d="${direction}"]`).boundingBox();
      await page.mouse.move(box.x+box.width/2,box.y+box.height/2);await page.mouse.down();
      await until(async()=> (await state()).v*sign>.1);
      await page.mouse.up();await until(async()=>Math.abs((await state()).v)<1e-6);
    }
    await page.locator('#resetOdom').click();
    await until(async()=> /reset confirmed/.test(await page.locator('#operationStatus').textContent()));
    await until(async()=>Math.abs(Number(await page.locator('#x').textContent()))<.03 && Math.abs(Number(await page.locator('#yaw').textContent()))<1.);
    await page.locator('#resetMap').click();
    await until(async()=> /Map cleared/.test(await page.locator('#operationStatus').textContent()));
    await page.locator('#goalX').fill('.25');await page.locator('#goalY').fill('0');await page.locator('#goalYaw').fill('10');
    await page.locator('#sendGoal').click();
    await until(async()=> (await state()).v>.01 || Math.abs((await state()).w)>.01);
    await until(async()=> /Goal reached/.test(await page.locator('#navStatus').textContent()),18000);
    await page.locator('#goalX').fill('.5');await page.locator('#sendGoal').click();
    await until(async()=> (await state()).v>.01);
    await page.locator('#navStop').click();await until(async()=>Math.abs((await state()).v)<1e-6);

    // Import the room image at the current physical pose, not just at origin.
    const png=await (await context.request.get(base+'/__test_map.png')).body();
    await page.locator('#knownMapFile').setInputFiles({name:'room.png',mimeType:'image/png',buffer:png});
    await page.locator('#knownMapResolution').fill('.10');await page.locator('#knownMapRadius').fill('6');
    await page.locator('#loadKnownMap').click();
    await page.waitForFunction(()=>document.getElementById('sendGoal').disabled);
    await until(async()=> !(await page.locator('#sendGoal').isDisabled()),31000);
    assert.match(await page.locator('#operationStatus').textContent(),/localized and ready/);
    assert.equal((await state()).mode,'known');
    await page.locator('#liveMapMode').click();
    await until(async()=> (await state()).mode==='live');
    await until(async()=> /Live mapping enabled/.test(await page.locator('#operationStatus').textContent()));
    assert.deepEqual((await state()).errors,[]);assert.deepEqual(pageErrors,[]);
    fs.mkdirSync(path.join(root,'tests/results'),{recursive:true});
    await page.screenshot({path:path.join(root,'tests/results/pipeline-browser.png')});
    console.log('Chrome full pipeline: manual turns/movement, confirmed resets/clear, goal arrival/cancel, PNG map import at a moved pose and live mapping — PASS');
  } finally {
    if(browser)await browser.close();
    child.stdin.write('stop\n');
    await new Promise(resolve=>{
      if(child.exitCode!==null)return resolve();
      const timer=setTimeout(()=>{child.kill();resolve();},5000);
      child.once('exit',()=>{clearTimeout(timer);resolve();});
    });
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
