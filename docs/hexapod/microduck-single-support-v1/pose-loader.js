// Each selected source-bound trajectory loads independently and caches offline.
const PREFIX='/hexapod/microduck-single-support-v1/';
const DIGEST=/^[0-9a-f]{64}$/;
const PIN='bae39f5900b318f7e87fc485c30a31213cfa14eb';
export function validateIndex(doc,geometryDigest){
 if(doc?.schema!=='microduck-single-support-replays/v1'||doc.model_commit!==PIN||doc.model_package_sha256!==geometryDigest||doc.frame_stride!==498||doc.visible_mesh_count!==70||!DIGEST.test(doc.source_summary_sha256)||!DIGEST.test(doc.source_verification_sha256)||!Array.isArray(doc.clips)||doc.clips.length!==14)throw Error('Некорректный каталог равновесия');
 const seen=new Set();
 for(const c of doc.clips){
  if(!/^[a-z0-9-]{1,90}$/.test(c.id)||seen.has(c.id)||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||!DIGEST.test(c.package_sha256)||!DIGEST.test(c.track_sha256)||!Number.isInteger(c.package_size)||c.package_size<50||c.package_size>1_500_000||c.url!==PREFIX+'tracks-'+c.package_sha256.slice(0,16)+'.json'||![360,410].includes(c.frames)||c.encoding!=='gzip-xor-byte-shuffle-v1'||c.decoded_size!==16+c.frames*498*4||!Number.isInteger(c.size)||c.size<50||c.size>c.decoded_size||!['CONTROL','FAIL','PASS'].includes(c.status)||Math.abs(c.duration_s-c.frames/50)>1e-5||c.terrain?.height_m!==0||!['stand','walk','left','right'].includes(c.group)||typeof c.quality_gate!=='boolean'||typeof c.single_support_1s!=='boolean'||!Array.isArray(c.observations)||c.observations.length!==c.frames||!c.observations.every(x=>Array.isArray(x)&&x.length===7&&x.every(Number.isFinite))||!Array.isArray(c.phase_events)||!c.phase_events.every(x=>Number.isFinite(x.t)&&x.t>=0&&x.t<=c.duration_s)||!['left','right'].every(s=>Number.isFinite(c.support_intervals_s?.[s])))throw Error('Повреждена запись равновесия');
  if((c.group==='stand'||c.group==='walk')!==(c.status==='CONTROL'))throw Error('Несогласованный контрольный сценарий');
  if(c.status==='PASS'&&(!c.single_support_1s||!c.quality_gate))throw Error('Несогласованный результат удержания');
  seen.add(c.id);
 }
 if(doc.default_clip!=='walk030-seed20261109'||!seen.has(doc.default_clip))throw Error('Нет начального сценария');
 return doc;
}
async function unpackGzip(encoded,limit){
  const reader=new Blob([encoded]).stream().pipeThrough(new DecompressionStream('gzip')).getReader();
  const chunks=[];let total=0;
  try{while(true){const {done,value}=await reader.read();if(done)break;total+=value.byteLength;if(total>limit)throw Error('Превышен размер распакованной записи');chunks.push(value);}}
  catch(e){await reader.cancel().catch(()=>{});throw e;}
  if(total!==limit)throw Error('Неверный размер распакованной записи');
  const all=new Uint8Array(total);let offset=0;for(const c of chunks){all.set(c,offset);offset+=c.byteLength;}return all.buffer;
}
function restoreTrack(encoded,frames){
  const input=new Uint8Array(encoded),count=frames*498,raw=new Uint8Array(input.length);
  raw.set(input.subarray(0,16));
  for(let i=0;i<count;i++)for(let b=0;b<4;b++)raw[16+i*4+b]=input[16+b*count+i];
  const view=new DataView(raw.buffer),stride=498*4;
  for(let row=1;row<frames;row++)for(let channel=0;channel<498;channel++){
    const at=16+row*stride+channel*4;
    view.setUint32(at,view.getUint32(at,true)^view.getUint32(at-stride,true),true);
  }
  return raw.buffer;
}
export async function decodePoseBytes(encoded,clip,index){
  if(encoded.byteLength!==clip.size||clip.decoded_size!==16+clip.frames*index.frame_stride*4)throw Error('Неверный размер физической записи');
  const hash=async bytes=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
  if(await hash(encoded)!==clip.sha256)throw Error('Контрольная сумма физической записи не совпадает');
  const bytes=restoreTrack(await unpackGzip(encoded,clip.decoded_size),clip.frames);
  if(await hash(bytes)!==clip.track_sha256)throw Error('Контрольная сумма кадров не совпадает');
  const head=new DataView(bytes);
  if(new TextDecoder().decode(bytes.slice(0,8))!=='MDSSUPV1'||
     head.getUint32(8,true)!==clip.frames||head.getUint32(12,true)!==498)
    throw Error('Неподдерживаемый формат физической записи');
  const floats=new Float32Array(clip.frames*498);
  for(let i=0;i<floats.length;i++)floats[i]=head.getFloat32(16+i*4,true);
  const frames=[];let previous=0;
  for(let n=0;n<clip.frames;n++){
    const f=floats.subarray(n*498,(n+1)*498);
    if(!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');
    for(let j=0;j<70;j++){
      const k=8+j*7+3,norm=Math.hypot(f[k],f[k+1],f[k+2],f[k+3]);
      if(Math.abs(norm-1)>.001)throw Error('Повреждён поворот детали');
    }
    previous=f[0];frames.push(f);
  }
  if(Math.abs(previous-clip.duration_s)>1e-5)throw Error('Неверная длительность записи');
  return {...clip,duration_s:clip.duration_s,frame_count:clip.frames,frames,gate_stable_at_end:clip.single_support_1s};
}
async function decodePackage(bytes,clip){
  if(bytes.byteLength!==clip.package_size)throw Error('Неверный размер пакета записей');
  const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
  if(hash!==clip.package_sha256)throw Error('Контрольная сумма пакета записей не совпадает');
  const doc=JSON.parse(new TextDecoder().decode(bytes));
  const record=doc.tracks?.[clip.track_sha256];
  if(doc.schema!=='microduck-single-support-track-package/v1'||!record||record.encoding!==clip.encoding||record.sha256!==clip.sha256||record.decoded_size!==clip.decoded_size||typeof record.data!=='string'||record.data.length>Math.ceil(clip.size/3)*4)throw Error('Некорректная запись в пакете');
  const text=atob(record.data),encoded=new Uint8Array(text.length);
  for(let i=0;i<text.length;i++)encoded[i]=text.charCodeAt(i);
  return encoded.buffer;
}
export async function loadPoseClip(clip,index){
  const cache=globalThis.caches?await caches.open('alfred-microduck-github-single-support-v1').catch(()=>null):null;
  if(cache){const old=await cache.match(clip.url);if(old){try{return await decodePoseBytes(await decodePackage(await old.arrayBuffer(),clip),clip,index);}catch{await cache.delete(clip.url);}}}
  const response=await fetch(clip.url,{cache:'no-store'});
  if(!response.ok)throw Error('Пакет записи недоступен: HTTP '+response.status);
  const reader=response.body.getReader(),chunks=[];let size=0;
  try{while(true){const {done,value}=await reader.read();if(done)break;size+=value.byteLength;if(size>clip.package_size)throw Error('Превышен размер пакета');chunks.push(value);}}
  catch(e){await reader.cancel().catch(()=>{});throw e;}
  const bytes=new Uint8Array(size);let at=0;for(const value of chunks){bytes.set(value,at);at+=value.byteLength;}
  const result=await decodePoseBytes(await decodePackage(bytes.buffer,clip),clip,index);
  if(cache)await cache.put(clip.url,new Response(bytes,{headers:{'Content-Type':'application/json'}})).catch(()=>{});
  return result;
}
