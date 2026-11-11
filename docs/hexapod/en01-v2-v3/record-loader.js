const DIGEST=/^[0-9a-f]{64}$/;
const PREFIX='/assets/robot-replays/en01-v2-v3/';
const PIN='9156683a0bb694c84dc541a3e26b5f6b08db1a41';
export function validateIndex(index){
  if(index?.schema!=='en01-recorded-experiments/v1'||index.step_git_blob_sha1!==PIN||index.frame_count!==300||index.frame_stride!==70||index.duration_s!==6||!DIGEST.test(index.cad_sha256)||!Number.isInteger(index.cad_size)||index.cad_size<1||index.cad_size>8000000||index.clips?.length!==37)throw Error('Неверный каталог EN01');
  const seen=new Set();
  for(const c of index.clips){
    if(!/^v[23]-[a-z0-9-]+$/.test(c.id)||seen.has(c.id)||!['v2','v3'].includes(c.version)||!['stand','push','drive','crouch','jump','fall'].includes(c.mode)||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||!DIGEST.test(c.model_sha256)||!Number.isInteger(c.size)||c.size<1||c.size>2000000||!c.url.startsWith(PREFIX)||!/^en01-[a-z0-9-]+\.json$/.test(c.url.slice(PREFIX.length))||!['research','failed'].includes(c.status)||!Number.isFinite(c.max_abs_x_m)||!Number.isFinite(c.max_abs_pitch_deg))throw Error('Повреждена ссылка на запись EN01');
    seen.add(c.id);
  }
  if(index.clips.filter(c=>c.version==='v2').length!==6||index.clips.filter(c=>c.version==='v3').length!==31||!seen.has(index.default_clip))throw Error('Неполный набор EN01');
  return index;
}
export async function checkedBytes(bytes,size,sha){
  if(bytes.byteLength!==size)throw Error('Неверный размер записи');
  const actual=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
  if(actual!==sha)throw Error('Контрольная сумма не совпала');return bytes;
}
export async function decodeRecord(bytes,clip,index){
  await checkedBytes(bytes,clip.size,clip.sha256);const d=JSON.parse(new TextDecoder().decode(bytes));
  if(d.format!=='navbot-en01-fourbar-physical-replay-v2'||d.physics_engine!=='MuJoCo'||d.physics_version!=='3.14.0'||d.frame_count!==300||d.frames?.length!==300||d.sample_stride!==70||d.duration_s!==6||d.status!==clip.status||d.source?.trace_sha256!==clip.trace_sha256||d.source.model_sha256!==clip.model_sha256||d.source.step_git_blob_sha1!==PIN||d.result.mode!==clip.mode||d.result.max_abs_x_m!==clip.max_abs_x_m||d.result.max_abs_pitch_deg!==clip.max_abs_pitch_deg||d.result.physics_steps!==6000)throw Error('Запись не соответствует исходному опыту');
  for(const key of ['body_names','rest_body_positions','part_body_map','cad_asset'])if(JSON.stringify(d[key])!==JSON.stringify(index.rig[key]))throw Error('Неверная кинематическая сборка');
  let previous=-1;
  for(const f of d.frames){if(f.length!==70||!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');for(let j=0;j<9;j++)if(Math.abs(Math.hypot(...f.slice(4+j*7,8+j*7))-1)>1e-8)throw Error('Неверный поворот звена');previous=f[0];}
  if(Math.abs(previous-5.981)>1e-8)throw Error('Неполная запись');return d;
}
export async function loadVerified(url,size,sha,decode){
  const cache=globalThis.caches?await caches.open('alfred-en01-recordings-v1').catch(()=>null):null;
  if(cache){const hit=await cache.match(url);if(hit){try{const b=await hit.arrayBuffer();await checkedBytes(b,size,sha);return await decode(b);}catch{await cache.delete(url);}}}
  const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
  const bytes=await r.arrayBuffer();await checkedBytes(bytes,size,sha);const result=await decode(bytes);
  if(cache)await cache.put(url,new Response(bytes)).catch(()=>{});return result;
}
