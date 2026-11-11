const DIGEST=/^[0-9a-f]{64}$/;
const PREFIX='/assets/robot-replays/en01-contact-v5/';
const PIN='9156683a0bb694c84dc541a3e26b5f6b08db1a41';
const CASES=['stand','crouch','forced-fold','forced-extend','jump-p1','jump-p2','jump-p5','jump-p10','jump-p5-halfdt'];
export function validateIndex(index){
 if(index?.schema!=='en01-cad-contact-recordings/v1'||index.step_git_blob_sha1!==PIN||index.frame_count!==300||index.frame_stride!==70||index.duration_s!==6||!DIGEST.test(index.cad_sha256)||!Number.isInteger(index.cad_size)||index.cad_size<1||index.cad_size>8000000||index.clips?.length!==9)throw Error('Неверный каталог EN01 v5');
 const seen=new Set();
 for(const c of index.clips){
  if(!CASES.includes(c.case)||c.id!=='v5-'+c.case||seen.has(c.id)||!['stand','crouch','forced-fold','forced-extend','jump'].includes(c.mode)||![1,2,5,10].includes(c.loop_ms)||c.anchor_q!==.1||![.00025,.000125].includes(c.timestep_s)||c.physics_steps!==Math.round(6/c.timestep_s)||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||!DIGEST.test(c.model_sha256)||!Number.isInteger(c.size)||c.size<1||c.size>2000000||!c.url.startsWith(PREFIX)||!/^en01-contact-v5-[a-z0-9-]+\.json$/.test(c.url.slice(PREFIX.length))||!['research','failed'].includes(c.status)||![c.max_abs_x_m,c.max_abs_pitch_deg,c.clearance_mm,c.flight_ms,c.landing_s].every(Number.isFinite)||typeof c.jump_pass!=='boolean'||typeof c.qualified_takeoff!=='boolean'||typeof c.upright_6s!=='boolean')throw Error('Повреждена запись EN01 v5');
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
 await checkedBytes(bytes,clip.size,clip.sha256);const d=JSON.parse(new TextDecoder().decode(bytes)),s=d.result,c=d.contact_repair;
 if(d.format!=='navbot-en01-fourbar-physical-replay-v2'||d.physics_engine!=='MuJoCo'||d.physics_version!=='3.14.0'||d.frame_count!==300||d.frames?.length!==300||d.sample_stride!==70||d.duration_s!==6||d.status!==clip.status||d.source?.trace_sha256!==clip.trace_sha256||d.source.model_sha256!==clip.model_sha256||d.source.step_git_blob_sha1!==PIN||s.experiment!=='cad_contact_v5'||s.mode!==clip.mode||s.physics_steps!==clip.physics_steps||s.timestep_s!==clip.timestep_s||s.max_abs_x_m!==clip.max_abs_x_m||s.max_abs_pitch_deg!==clip.max_abs_pitch_deg)throw Error('Запись не соответствует исходному опыту');
 if(c?.version!==5||!c.all_step_hip_limits_ok||c.hardware_jump_claim_allowed!==false||c.cad_surface_audit?.unexpected_intersection_frames!==0||c.cad_surface_audit.saved_frames!==300||s.self_collision!=='CAD_CONVEX_OUTER_ENVELOPES'||Math.min(...s.hip_min_all_steps_rad)<0||Math.max(...s.hip_max_all_steps_rad)>.65)throw Error('Нет проверки механики');
 if(d.source_command_experiment?.loop_period_ms_ASSUMED!==clip.loop_ms||d.source_command_experiment.cad_anchor_q_rad_ASSUMED!==.1||s.jump_pass!==clip.jump_pass||s.qualified_flight!==clip.qualified_takeoff||s.stable_landing_s!==clip.landing_s||1000*s.max_min_wheel_clearance_m!==clip.clearance_mm||1000*Math.max(0,...s.flights.filter(f=>f.qualified).map(f=>f.duration_s))!==clip.flight_ms)throw Error('Подменены условия или метрики');
 for(const key of ['body_names','rest_body_positions','part_body_map','cad_asset'])if(JSON.stringify(d[key])!==JSON.stringify(index.rig[key]))throw Error('Неверная кинематическая сборка');
 let previous=-1;
 for(const f of d.frames){if(f.length!==70||!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');for(let j=0;j<9;j++)if(Math.abs(Math.hypot(...f.slice(4+j*7,8+j*7))-1)>1e-8)throw Error('Неверный поворот звена');previous=f[0];}
 if(Math.abs(previous-(5.98+clip.timestep_s))>1e-8)throw Error('Неполная запись');return d;
}
export async function loadVerified(url,size,sha,decode){
 const cache=globalThis.caches?await caches.open('alfred-en01-cad-contact-v5').catch(()=>null):null;
 if(cache){const hit=await cache.match(url);if(hit){try{const b=await hit.arrayBuffer();await checkedBytes(b,size,sha);return await decode(b);}catch{await cache.delete(url);}}}
 const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
 const bytes=await r.arrayBuffer();await checkedBytes(bytes,size,sha);const result=await decode(bytes);
 if(cache)await cache.put(url,new Response(bytes)).catch(()=>{});return result;
}
