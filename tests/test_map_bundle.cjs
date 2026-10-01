const fs=require('node:fs');
const assert=require('node:assert/strict');
const {test}=require('node:test');
const code=fs.readFileSync(require('node:path').join(__dirname,'../js/map-exporter.js'),'utf8');
const modulePromise=import('data:text/javascript;base64,'+Buffer.from(code).toString('base64'));
const nav={zones:[{id:'a',name:'Livraison café',x:1,y:-2,radius:.4}],base:{x:0,y:0,yaw_deg:90}};
test('JSON export/import preserves exact occupancy, scale, UTF-8 zones and base',async()=>{
  const m=await modulePromise;
  const original=m.MapExporter.bundle({resolution:1,radius:1,grid:new Int8Array([-1,0,100,0])},nav);
  assert.deepEqual(m.validateBundle(JSON.parse(JSON.stringify(original))),{resolution:1,radius:1,grid:[-1,0,100,0],navigation_config:nav});
});
test('PNG annotation chunk preserves UTF-8 geometry without altering image chunks',async()=>{
  const m=await modulePromise;
  const png=Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+/l9sAAAAASUVORK5CYII=','base64');
  const metadata={format:'fabtino-map',version:1,resolution:.1,radius:6,navigation_config:nav};
  const encoded=m.embedPNG(png,metadata);
  assert.deepEqual(m.readPNGMetadata(encoded),metadata);
  assert.deepEqual(Buffer.from(encoded.subarray(0,png.length-12)),png.subarray(0,png.length-12));
  assert.equal(m.readPNGMetadata(png),null);
  encoded[png.length+15]^=1;
  assert.throws(()=>m.readPNGMetadata(encoded),/checksum/);
});
test('invalid occupancy, unbounded dimensions, zone radii and duplicate names fail before import',async()=>{
  const m=await modulePromise;
  for(const bad of [0,-1,NaN,Infinity]) assert.throws(()=>m.validateNavigation({zones:[{...nav.zones[0],radius:bad}]}));
  assert.throws(()=>m.validateNavigation({zones:[nav.zones[0],{...nav.zones[0],id:'b'}]}));
  assert.throws(()=>m.validateBundle({format:'fabtino-map',version:1,resolution:.001,radius:100,grid:[]}));
  assert.throws(()=>m.validateBundle({format:'fabtino-map',version:1,resolution:1,radius:1,grid:[0,0,0,50]}));
});
