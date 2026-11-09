const DIGEST=/^[0-9a-f]{64}$/;
const PREFIX='/assets/robot-replays/en01-getup-v7/';
const PIN='9156683a0bb694c84dc541a3e26b5f6b08db1a41';
const ORIENTATIONS=['front','back','left','right'];
const STRATEGIES=['idle','extend-forward','extend-backward','rock','asymmetric'];
export function validateIndex(index){
 if(index?.schema!=='en01-getup-recordings/v1'||index.step_git_blob_sha1!==PIN||index.frame_count!==300||index.frame_stride!==70||index.duration_s!==6||!DIGEST.test(index.model_sha256)||!DIGEST.test(index.cad_sha256)||!Number.isInteger(index.cad_size)||index.cad_size<1||index.cad_size>8000000||index.clips?.length!==20)throw Error('Неверный каталог EN01 v7');
 const seen=new Set();
 for(const c of index.clips){
  if(!ORIENTATIONS.includes(c.orientation)||!STRATEGIES.includes(c.strategy)||c.id!=='v7-'+c.orientation+'-'+c.strategy||seen.has(c.id)||c.mode!=='getup'||c.timestep_s!==.00025||c.physics_steps!==24000||c.model_sha256!==index.model_sha256||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||!Number.isInteger(c.size)||c.size<1||c.size>2000000||!c.url.startsWith(PREFIX)||!/^en01-getup-v7-[a-z0-9-]+\.json$/.test(c.url.slice(PREFIX.length))||!['research','failed'].includes(c.status)||typeof c.getup_pass!=='boolean'||typeof c.geometry_ok!=='boolean'||!c.metrics||!['final_stable_s','longest_stable_s','final_tilt_deg','tilt_at_command_deg'].every(k=>Number.isFinite(c.metrics[k])))throw Error('Повреждена запись EN01 v7');
  if(c.status!==((c.getup_pass&&c.geometry_ok)?'research':'failed'))throw Error('Неверный результат');seen.add(c.id);
 }
 if(!seen.has(index.default_clip))throw Error('Нет начальной записи');return index;
}
export async function checkedBytes(bytes,size,sha){
 if(bytes.byteLength!==size)throw Error('Неверный размер записи');
 const actual=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
 if(actual!==sha)throw Error('Контрольная сумма не совпала');return bytes;
}
export async function decodeRecord(bytes,clip,index){
 await checkedBytes(bytes,clip.size,clip.sha256);const d=JSON.parse(new TextDecoder().decode(bytes)),s=d.result,c=d.getup;
 if(d.format!=='navbot-en01-fourbar-physical-replay-v2'||d.physics_engine!=='MuJoCo'||d.physics_version!=='3.14.0'||d.frame_count!==300||d.frames?.length!==300||d.sample_stride!==70||d.duration_s!==6||d.status!==clip.status||d.source?.trace_sha256!==clip.trace_sha256||d.source.model_sha256!==index.model_sha256||d.source.step_git_blob_sha1!==PIN||s.experiment!=='getup_v7'||s.orientation!==clip.orientation||s.strategy!==clip.strategy||s.mode!=='getup'||s.physics_steps!==24000||s.timestep_s!==.00025)throw Error('Запись не соответствует исходному опыту');
 if(c?.version!==7||c.orientation!==clip.orientation||c.strategy!==clip.strategy||c.geometry_ok!==clip.geometry_ok||c.passed!==clip.getup_pass||s.hardware_motion_claim_allowed!==false||s.getup_pass!==clip.getup_pass)throw Error('Подменён результат опыта');
 for(const [key,value] of Object.entries(clip.metrics))if(s[key]!==value)throw Error('Подменены метрики');
 for(const key of ['body_names','rest_body_positions','part_body_map','cad_asset'])if(JSON.stringify(d[key])!==JSON.stringify(index.rig[key]))throw Error('Неверная кинематическая сборка');
 let previous=-1;
 for(const f of d.frames){if(f.length!==70||!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');for(let j=0;j<9;j++)if(Math.abs(Math.hypot(...f.slice(4+j*7,8+j*7))-1)>1e-8)throw Error('Неверный поворот звена');previous=f[0];}
 if(Math.abs(previous-5.98025)>1e-8)throw Error('Неполная запись');return d;
}
export async function loadVerified(url,size,sha,decode){
 const cache=globalThis.caches?await caches.open('alfred-en01-getup-v7').catch(()=>null):null;
 if(cache){const hit=await cache.match(url);if(hit){try{const b=await hit.arrayBuffer();await checkedBytes(b,size,sha);return await decode(b);}catch{await cache.delete(url);}}}
 const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
 const bytes=await r.arrayBuffer();await checkedBytes(bytes,size,sha);const result=await decode(bytes);
 if(cache)await cache.put(url,new Response(bytes)).catch(()=>{});return result;
}
