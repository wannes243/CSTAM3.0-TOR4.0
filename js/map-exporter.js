// Pixels remain occupancy data; annotations travel as PNG metadata or JSON.
export function validateNavigation(value = {}) {
  const zones = value.zones ?? [];
  if (!Array.isArray(zones) || zones.length > 64) throw new Error('At most 64 zones are allowed.');
  const ids = new Set(), names = new Set();
  const number = v => { if (typeof v !== 'number' || !Number.isFinite(v)) throw new Error('Coordinates and radii must be finite numbers.'); return v; };
  const normalized = zones.map(z => {
    if (!z || typeof z.id !== 'string' || !z.id.trim() || z.id.length > 128 || ids.has(z.id)) throw new Error('Zone IDs must be unique.');
    if (typeof z.name !== 'string' || !z.name.trim() || z.name.trim().length > 60 || names.has(z.name.trim().toLowerCase())) throw new Error('Give every zone a unique name (1–60 characters).');
    ids.add(z.id); names.add(z.name.trim().toLowerCase());
    const radius = number(z.radius);
    if (radius < .05 || radius > 100) throw new Error('Zone radius must be between 0.05 and 100 m.');
    return { id: z.id, name: z.name.trim(), x: number(z.x), y: number(z.y), radius };
  });
  const base = value.base == null ? null : { x: number(value.base.x), y: number(value.base.y), yaw_deg: number(value.base.yaw_deg ?? 0) };
  return { zones: normalized, base };
}
export function validateBundle(value) {
  if (value?.format !== 'fabtino-map' || value.version !== 1) throw new Error('Unsupported map bundle.');
  const { resolution, radius, grid } = value, size = Math.ceil(2 * radius / resolution);
  if (!Number.isFinite(resolution) || !Number.isFinite(radius) || resolution <= 0 || radius <= 0 || size * size > 1000000 || !Array.isArray(grid) || grid.length !== size * size || grid.some(v => ![-1, 0, 100].includes(v))) throw new Error('Invalid map dimensions or occupancy cells.');
  return { resolution, radius, grid, navigation_config: validateNavigation(value.navigation_config) };
}
const signature = [137,80,78,71,13,10,26,10];
function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) { crc ^= byte; for (let b = 0; b < 8; b++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0); }
  return (crc ^ 0xffffffff) >>> 0;
}
function chunks(bytes) {
  if (!signature.every((v,i) => bytes[i] === v)) return [];
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength), result = [];
  for (let offset = 8; offset < bytes.length;) {
    if (offset + 12 > bytes.length) throw new Error('Truncated PNG.');
    const length = view.getUint32(offset);
    if (offset + 12 + length > bytes.length) throw new Error('Invalid PNG chunk.');
    const type = String.fromCharCode(...bytes.subarray(offset+4,offset+8));
    result.push({offset,length,type}); offset += length + 12;
    if (type === 'IEND') break;
  }
  return result;
}
export function embedPNG(bytes, metadata) {
  const end = chunks(bytes).find(c => c.type === 'IEND');
  if (!end) throw new Error('PNG has no end chunk.');
  const data = new TextEncoder().encode('fabtino_navigation\0\0\0\0\0' + JSON.stringify(metadata));
  const chunk = new Uint8Array(data.length+12), view = new DataView(chunk.buffer);
  view.setUint32(0,data.length); chunk.set([105,84,88,116],4); chunk.set(data,8);
  view.setUint32(chunk.length-4,crc32(chunk.subarray(4,chunk.length-4)));
  const output = new Uint8Array(bytes.length+chunk.length);
  output.set(bytes.subarray(0,end.offset)); output.set(chunk,end.offset); output.set(bytes.subarray(end.offset),end.offset+chunk.length);
  return output;
}
export function readPNGMetadata(bytes) {
  for (const c of chunks(bytes)) {
    if (c.type !== 'iTXt') continue;
    const data = bytes.subarray(c.offset+8,c.offset+8+c.length), keywordEnd = data.indexOf(0);
    if (new TextDecoder().decode(data.subarray(0,keywordEnd)) !== 'fabtino_navigation') continue;
    const view = new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
    if (crc32(bytes.subarray(c.offset+4,c.offset+8+c.length)) !== view.getUint32(c.offset+8+c.length)) throw new Error('Map metadata checksum failed.');
    if (data[keywordEnd+1] !== 0 || data[keywordEnd+2] !== 0) throw new Error('Unsupported map metadata compression.');
    const languageEnd = data.indexOf(0,keywordEnd+3), titleEnd = data.indexOf(0,languageEnd+1);
    if (languageEnd < 0 || titleEnd < 0) throw new Error('Invalid map metadata.');
    return JSON.parse(new TextDecoder().decode(data.subarray(titleEnd+1)));
  }
  return null;
}
function download(blob,name) {
  const url = URL.createObjectURL(blob), link = document.createElement('a');
  link.download = name; link.href = url; link.click(); setTimeout(() => URL.revokeObjectURL(url),1000);
}
export const MapExporter = {
  bundle(map, navigation) { return { format: 'fabtino-map', version: 1, resolution: map.resolution, radius: map.radius, grid: Array.from(map.grid), navigation_config: validateNavigation(navigation) }; },
  exportJSON(map,navigation) { download(new Blob([JSON.stringify(this.bundle(map,navigation))],{type:'application/json'}),'fabtino_map.json'); },
  async exportPNG(map,navigation) {
    const canvas = document.createElement('canvas'); canvas.width = canvas.height = map.size;
    const ctx = canvas.getContext('2d'), pixels = ctx.createImageData(map.size,map.size);
    for (let y=0;y<map.size;y++) for(let x=0;x<map.size;x++) {
      const v=map.grid[y*map.size+x], color=v<0?128:v===100?0:255, i=((map.size-1-y)*map.size+x)*4;
      pixels.data.set([color,color,color,255],i);
    }
    ctx.putImageData(pixels,0,0);
    const blob = await new Promise(resolve => canvas.toBlob(resolve,'image/png'));
    if (!blob) throw new Error('Could not encode the map.');
    const metadata = { format:'fabtino-map', version:1, resolution:map.resolution, radius:map.radius, navigation_config:validateNavigation(navigation) };
    download(new Blob([embedPNG(new Uint8Array(await blob.arrayBuffer()),metadata)],{type:'image/png'}),'fabtino_2d_map.png');
  }
};
