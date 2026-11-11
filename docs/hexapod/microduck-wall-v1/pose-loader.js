// Compact source-bound FK tracks; one on-demand download per selected clip.
const PREFIX='/assets/robot-replays/microduck-contact-v2/';
const DIGEST=/^[0-9a-f]{64}$/;
const PIN='bae39f5900b318f7e87fc485c30a31213cfa14eb';
export function validateIndex(doc,geometryDigest){
 if(doc?.schema!=='microduck-body-terrain-replays/v2'||doc.model_commit!==PIN||doc.model_package_sha256!==geometryDigest||doc.stand_policy_sha256!=='2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492'||!DIGEST.test(doc.source_summary_sha256)||(!DIGEST.test(doc.collision_model_sha256)||doc.environment_mesh_count!==60)||doc.frame_stride!==498||doc.visible_mesh_count!==70||!Array.isArray(doc.clips)||doc.clips.length!==9)throw Error('Некорректный каталог заваливаний');
 const seen=new Set();
 for(const c of doc.clips){
  if(!/^(?:close-h(?:10|20|30|50|100|250)|reference-h(?:10|20)|flat-h0)$/.test(c.id)||seen.has(c.id)||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||c.url!==PREFIX+c.id+'-'+c.sha256.slice(0,12)+'.bin'||!Number.isInteger(c.frames)||c.frames!==500||c.size!==16+c.frames*498*4||!['PASS','FAIL'].includes(c.status)||!Number.isFinite(c.duration_s)||c.duration_s!==10||!['complete_env_05ms'].includes(c.contact_profile)||![0,.01,.02,.03,.05,.1,.25].includes(c.terrain?.height_m)||(!Number.isFinite(c.terrain.edge_x_m)||c.terrain.edge_x_m<.02||c.terrain.edge_x_m>.08)||!Array.isArray(c.phase_events)||typeof c.directed_fall!=='boolean'||typeof c.step_goal_reached!=='boolean'||typeof c.fall_and_recover!=='boolean'||typeof c.standing_final_1s!=='boolean'||(!Number.isFinite(c.peak_directional_lean_deg)||!Number.isFinite(c.surface_gap_before_trigger_m)||!Number.isFinite(c.rear_body_contact_s)||!['close','reference','flat'].includes(c.condition)||typeof c.supported_top_stance!=='boolean'||!Number.isFinite(c.max_sole_overhang_m)))throw Error('Повреждена ссылка на испытание');
  seen.add(c.id);
 }
 return doc;
}
export async function decodePoseBytes(bytes,clip,index){
  if(bytes.byteLength!==clip.size||bytes.byteLength!==16+clip.frames*index.frame_stride*4)
    throw Error('Неверный размер физической записи');
  const digest=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),
    b=>b.toString(16).padStart(2,'0')).join('');
  if(digest!==clip.sha256)throw Error('Контрольная сумма физической записи не совпадает');
  const head=new DataView(bytes);
  if(new TextDecoder().decode(bytes.slice(0,8))!=='MDWALL2\0'||
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
  return {...clip,duration_s:clip.duration_s,frame_count:clip.frames,frames,gate_stable_at_end:clip.stable_at_end_1s};
}
export async function loadPoseClip(clip,index){
  const cache=globalThis.caches?await caches.open('alfred-microduck-contact-clips-v2').catch(()=>null):null;
  if(cache){
    const old=await cache.match(clip.url);
    if(old){try{return await decodePoseBytes(await old.arrayBuffer(),clip,index);}
      catch{await cache.delete(clip.url);}}
  }
  const r=await fetch(clip.url,{cache:'no-store'});
  if(!r.ok)throw Error('Запись недоступна: HTTP '+r.status);
  const reader=r.body.getReader(),chunks=[];let size=0;
  try{while(true){const {done,value}=await reader.read();if(done)break;
    size+=value.byteLength;if(size>clip.size)throw Error('Превышен размер записи');chunks.push(value);}}
  catch(e){await reader.cancel().catch(()=>{});throw e;}
  const all=new Uint8Array(size);let offset=0;for(const c of chunks){all.set(c,offset);offset+=c.byteLength;}
  const result=await decodePoseBytes(all.buffer,clip,index);
  if(cache)await cache.put(clip.url,new Response(all,{headers:{'Content-Type':'application/octet-stream'}})).catch(()=>{});
  return result;
}
