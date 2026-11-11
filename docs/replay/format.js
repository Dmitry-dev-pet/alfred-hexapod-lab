// Robot Replay v1: data-only, self-contained scene and measured world poses.
const LIMIT = 100 * 1024 * 1024;
const fail = message => { throw Error(message); };
const record = x => x && typeof x === 'object' && !Array.isArray(x);
const vector = (x, n) => Array.isArray(x) && x.length === n && x.every(Number.isFinite);
const hex = b => [...new Uint8Array(b)].map(v => v.toString(16).padStart(2,'0')).join('');
export async function sha256(bytes) { return hex(await crypto.subtle.digest('SHA-256', bytes)); }
async function inflate(bytes, limit = LIMIT) {
  const reader = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip')).getReader();
  const chunks = []; let size = 0;
  try { for (;;) { const {value, done} = await reader.read(); if (done) break; size += value.length;
    if (size > limit) fail('Распакованный файл слишком большой'); chunks.push(value); }
  } catch (e) { await reader.cancel().catch(()=>{}); throw e; }
  const out = new Uint8Array(size); let offset = 0;
  for (const chunk of chunks) { out.set(chunk, offset); offset += chunk.length; }
  return out;
}
async function decodeBlock(b, type, count) {
  const Type = {f32:Float32Array, f64:Float64Array, u32:Uint32Array}[type];
  if (!record(b) || b.encoding !== type+'le-gzip-base64' || !Number.isSafeInteger(count) || count < 1 ||
      b.count !== count || count * Type.BYTES_PER_ELEMENT > LIMIT || typeof b.data !== 'string' ||
      b.data.length > LIMIT || !/^[a-f0-9]{64}$/.test(b.sha256)) fail('Неверный блок данных');
  const compressed = Uint8Array.from(atob(b.data), c => c.charCodeAt(0));
  const raw = await inflate(compressed, count * Type.BYTES_PER_ELEMENT);
  if (raw.byteLength !== count * Type.BYTES_PER_ELEMENT || await sha256(raw) !== b.sha256) fail('Повреждён блок данных');
  if (new Uint8Array(new Uint32Array([1]).buffer)[0] !== 1) fail('Неподдерживаемый порядок байтов');
  const values = new Type(raw.buffer);
  if (type !== 'u32' && !values.every(Number.isFinite)) fail('Нечисловые данные');
  return values;
}
function validateSurface(s) {
  if (!record(s) || !['box','plane'].includes(s.kind) || !vector(s.size,3) || s.size.some(x=>x<=0) ||
      !vector(s.position,3) || !/^#[0-9a-f]{6}$/i.test(s.color)) fail('Неверное окружение');
}
export async function loadReplay(input, progress = ()=>{}) {
  let bytes = new Uint8Array(input);
  if (!bytes.length || bytes.length > LIMIT) fail('Неверный размер файла');
  if (bytes[0] === 31 && bytes[1] === 139) bytes = await inflate(bytes);
  const doc = JSON.parse(new TextDecoder('utf-8',{fatal:true}).decode(bytes));
  if (!record(doc) || doc.schema !== 'robot-replay/v1' || !record(doc.scene) || !record(doc.metadata) ||
      typeof doc.metadata.title !== 'string' || !doc.metadata.title.length || doc.metadata.title.length>200 ||
      !['Z','Y'].includes(doc.scene.up_axis) || doc.scene.units !== 'm' || !Array.isArray(doc.nodes) ||
      !doc.nodes.length || doc.nodes.length > 1000 || !Array.isArray(doc.meshes) || !doc.meshes.length ||
      doc.meshes.length>1000 || !Array.isArray(doc.clips) || !doc.clips.length || doc.clips.length>300) fail('Неподдерживаемый формат записи');
  const ids = new Set();
  for (const n of doc.nodes) { if (!record(n) || typeof n.id!=='string' || !n.id || ids.has(n.id)) fail('Неверные звенья'); ids.add(n.id); }
  const cam=doc.scene.camera;
  if (!record(cam)||!vector(cam.offset,3)||Math.hypot(...cam.offset)<.001||!Number.isInteger(cam.follow_node)||cam.follow_node<0||cam.follow_node>=doc.nodes.length) fail('Неверная камера');
  if (!/^#[0-9a-f]{6}$/i.test(doc.scene.background)||!Array.isArray(doc.scene.lights)||doc.scene.lights.length>10) fail('Неверная сцена');
  for(const l of doc.scene.lights) if(!record(l)||!['hemisphere','directional'].includes(l.kind)||!/^#[0-9a-f]{6}$/i.test(l.color)||!(l.intensity>=0&&l.intensity<=10)||!vector(l.position,3)||!/^#[0-9a-f]{6}$/i.test(l.ground)) fail('Неверное освещение');
  const meshes=[]; let totalDecoded=0;
  for(const m of doc.meshes) {
    if(!record(m)||!Number.isInteger(m.node)||m.node<0||m.node>=doc.nodes.length||!Number.isInteger(m.vertex_count)||m.vertex_count<3||m.vertex_count>5e6||!Number.isInteger(m.triangle_count)||m.triangle_count<1||m.triangle_count>5e6||!vector(m.translation,3)||!Number.isFinite(m.scale)||!(m.scale>0)||!/^#[0-9a-f]{6}$/i.test(m.material?.color)||!(m.material?.roughness>=0&&m.material.roughness<=1)||!(m.material?.metalness>=0&&m.material.metalness<=1)) fail('Неверная геометрия');
    totalDecoded+=(m.vertex_count+m.triangle_count)*12;
    if(totalDecoded>LIMIT) fail('Слишком большая сцена');
    const positions=await decodeBlock(m.positions,'f32',m.vertex_count*3), indices=await decodeBlock(m.indices,'u32',m.triangle_count*3);
    if(indices.some(i=>i>=m.vertex_count)) fail('Индекс вне геометрии');
    meshes.push({...m,positions,indices}); progress('Геометрия: '+meshes.length+' / '+doc.meshes.length);
  }
  const clips=[], clipIDs=new Set(), stride=1+7*doc.nodes.length;
  for(const c of doc.clips) {
    if(!record(c)||typeof c.id!=='string'||clipIDs.has(c.id)||typeof c.title!=='string'||!Number.isFinite(c.duration_s)||!(c.duration_s>0)||!Number.isInteger(c.frame_count)||c.frame_count<1||c.frame_count>1e6||!Array.isArray(c.environment)||c.environment.length>100||c.frame_stride!==stride) fail('Неверная запись движения');
    clipIDs.add(c.id); c.environment.forEach(validateSurface);
    totalDecoded+=c.frame_count*stride*8;
    if(totalDecoded>LIMIT) fail('Слишком много кадров');
    const frames=await decodeBlock(c.frames,'f64',c.frame_count*stride);
    let prev=-Infinity;
    for(let row=0;row<c.frame_count;row++) { const o=row*stride,t=frames[o];
      if(t<0||t<=prev||t>c.duration_s+1e-6) fail('Неверное время кадра'); prev=t;
      for(let j=0;j<doc.nodes.length;j++) { const q=o+4+7*j;
        if(Math.abs(Math.hypot(frames[q],frames[q+1],frames[q+2],frames[q+3])-1)>.0001) fail('Повреждён поворот'); }
    }
    let contacts=null;
    if(c.contacts) { const channels=c.contacts.labels;
      if(!Array.isArray(channels)||channels.length<1||channels.length>100||channels.some(x=>typeof x!=='string')) fail('Неверная контактная трасса');
      totalDecoded+=c.frame_count*channels.length*8;
      if(totalDecoded>LIMIT) fail('Слишком большая контактная трасса');
      contacts=await decodeBlock(c.contacts.values,'f64',c.frame_count*channels.length);
      if(contacts.some(x=>x<0)) fail('Отрицательная нормальная сила');
    }
    clips.push({...c,frames,contacts});
  }
  if(!clipIDs.has(doc.default_clip)) fail('Нет начального опыта');
  return {doc,meshes,clips};
}
export function sampleAt(clip,time) {
  const f=clip.frames,s=clip.frame_stride,last=clip.frame_count-1;
  if(time<=f[0])return {a:0,b:0,k:0};
  if(time>=f[last*s])return {a:last,b:last,k:0};
  let a=0,b=last;while(b-a>1){const m=(a+b)>>1;if(f[m*s]<=time)a=m;else b=m;}
  return {a,b,k:(time-f[a*s])/(f[b*s]-f[a*s])};
}
