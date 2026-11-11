const DIGEST=/^[0-9a-f]{64}$/;
const PREFIX='/assets/robot-replays/en01-basic-v6/';
const PIN='9156683a0bb694c84dc541a3e26b5f6b08db1a41';
const MODEL='04f24a31a9c1f72e6f8599c266876ec6bdf17f2819ed79f510d1c145c82663f3';
const CASES=['stand-low','stand-mid','drive-forward','drive-backward','turn-left-low','turn-right-low','turn-left-mid','turn-left-high','squat-cycle','squat-turn-left','squat-turn-right','drive-turn-left'];
const metricKeys=['signed_axle_travel_m','yaw_change_deg','max_abs_tilt_deg','final_stop_speed_m_s','final_stop_yaw_rate_rad_s','active_contact_slip_rms_m_s','squat_height_change_m'];
export function validateIndex(index){
 if(index?.schema!=='en01-basic-motion-recordings/v1'||index.step_git_blob_sha1!==PIN||index.frame_count!==300||index.frame_stride!==70||index.duration_s!==6||!DIGEST.test(index.cad_sha256)||!Number.isInteger(index.cad_size)||index.cad_size<1||index.cad_size>8000000||index.clips?.length!==12)throw Error('Неверный каталог EN01 v6');
 const seen=new Set();
 for(const c of index.clips){
  if(!CASES.includes(c.case)||c.id!=='v6-'+c.case||seen.has(c.id)||c.mode!=='basic'||c.timestep_s!==.00025||c.physics_steps!==24000||c.model_sha256!==MODEL||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||!Number.isInteger(c.size)||c.size<1||c.size>2000000||!c.url.startsWith(PREFIX)||!/^en01-basic-v6-[a-z0-9-]+\.json$/.test(c.url.slice(PREFIX.length))||!['research','failed'].includes(c.status)||typeof c.basic_pass!=='boolean'||typeof c.geometry_ok!=='boolean'||!c.checks||!Object.values(c.checks).every(v=>typeof v==='boolean')||!c.metrics||!metricKeys.every(k=>Number.isFinite(c.metrics[k])))throw Error('Повреждена запись EN01 v6');
  if(c.basic_pass!==Object.values(c.checks).every(Boolean)||c.status!==((c.basic_pass&&c.geometry_ok)?'research':'failed'))throw Error('Неверный результат проверки');
  seen.add(c.id);
 }
 if(!seen.has(index.default_clip))throw Error('Нет начальной записи');return index;
}
export async function checkedBytes(bytes,size,sha){
 if(bytes.byteLength!==size)throw Error('Неверный размер записи');
 const actual=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
 if(actual!==sha)throw Error('Контрольная сумма не совпала');return bytes;
}
export async function decodeRecord(bytes,clip,index){
 await checkedBytes(bytes,clip.size,clip.sha256);const d=JSON.parse(new TextDecoder().decode(bytes)),s=d.result,c=d.basic_motion;
 if(d.format!=='navbot-en01-fourbar-physical-replay-v2'||d.physics_engine!=='MuJoCo'||d.physics_version!=='3.14.0'||d.frame_count!==300||d.frames?.length!==300||d.sample_stride!==70||d.duration_s!==6||d.status!==clip.status||d.source?.trace_sha256!==clip.trace_sha256||d.source.model_sha256!==MODEL||d.source.step_git_blob_sha1!==PIN||s.experiment!=='basic_motion_v6'||s.case!==clip.case||s.mode!=='basic'||s.physics_steps!==24000||s.timestep_s!==.00025)throw Error('Запись не соответствует исходному опыту');
 if(c?.version!==6||c.case!==clip.case||c.geometry_ok!==clip.geometry_ok||c.hardware_motion_claim_allowed!==false||c.basic_pass!==clip.basic_pass||s.basic_pass!==clip.basic_pass||JSON.stringify(c.checks)!==JSON.stringify(clip.checks)||JSON.stringify(s.checks)!==JSON.stringify(clip.checks)||JSON.stringify(s.metrics)!==JSON.stringify(clip.metrics)||s.max_abs_x_m!==clip.max_abs_x_m||s.max_abs_pitch_deg!==clip.max_abs_pitch_deg)throw Error('Подменены условия или метрики');
 for(const key of ['body_names','rest_body_positions','part_body_map','cad_asset'])if(JSON.stringify(d[key])!==JSON.stringify(index.rig[key]))throw Error('Неверная кинематическая сборка');
 let previous=-1;
 for(const f of d.frames){if(f.length!==70||!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');for(let j=0;j<9;j++)if(Math.abs(Math.hypot(...f.slice(4+j*7,8+j*7))-1)>1e-8)throw Error('Неверный поворот звена');previous=f[0];}
 if(Math.abs(previous-5.98025)>1e-8)throw Error('Неполная запись');return d;
}
export async function loadVerified(url,size,sha,decode){
 const cache=globalThis.caches?await caches.open('alfred-en01-basic-v6').catch(()=>null):null;
 if(cache){const hit=await cache.match(url);if(hit){try{const b=await hit.arrayBuffer();await checkedBytes(b,size,sha);return await decode(b);}catch{await cache.delete(url);}}}
 const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
 const bytes=await r.arrayBuffer();await checkedBytes(bytes,size,sha);const result=await decode(bytes);
 if(cache)await cache.put(url,new Response(bytes)).catch(()=>{});return result;
}
