// Compact source-bound FK tracks; one on-demand download per selected clip.
const PREFIX='/assets/robot-replays/microduck-runup-roll-v1/';
const DIGEST=/^[0-9a-f]{64}$/;
const PIN='bae39f5900b318f7e87fc485c30a31213cfa14eb';
export function validateIndex(doc,geometryDigest){
 if(doc?.schema!=='microduck-runup-roll-replays/v1'||doc.model_commit!==PIN||doc.model_package_sha256!==geometryDigest||doc.roll_policy_sha256!=='3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b'||doc.stand_policy_sha256!=='2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492'||!DIGEST.test(doc.source_summary_sha256)||(!DIGEST.test(doc.collision_model_sha256)||doc.environment_mesh_count!==60)||doc.frame_stride!==498||doc.visible_mesh_count!==70||!Array.isArray(doc.clips)||doc.clips.length!==12)throw Error('Некорректный каталог заваливаний');
 const seen=new Set();
 for(const c of doc.clips){
  const ids=['runup-flat','runup-flat-moving','runup-6','runup-3','runup-1','runup-contact','runup-1-long','runup-contact-long','runup-contact-fast','runup-10-early','runup-1-roll3','runup-1-roll4'];
  if(!ids.includes(c.id)||seen.has(c.id)||!DIGEST.test(c.sha256)||!DIGEST.test(c.trace_sha256)||c.url!==PREFIX+c.id+'-'+c.sha256.slice(0,12)+'.bin'||!Number.isInteger(c.frames)||![500,700,800].includes(c.frames)||c.size!==16+c.frames*498*4||!['PASS','FAIL'].includes(c.status)||c.duration_s!==c.frames/50||c.contact_profile!=='complete_env_05ms'||![0,.05].includes(c.terrain?.height_m)||c.terrain.edge_x_m!==.25||!Array.isArray(c.phase_events)||!c.phase_events.every(e=>Number.isFinite(e.t)&&e.t>=0&&e.t<=c.duration_s&&['settle','approach','roll','recovery','hold'].includes(e.name))||typeof c.full_forward_roll!=='boolean'||typeof c.step_goal_reached!=='boolean'||typeof c.standing_final_1s!=='boolean'||typeof c.quality_gate!=='boolean'||!Number.isFinite(c.active_roll_max_forward_deg)||!Number.isFinite(c.pretrigger_mean_forward_mps)||(c.trigger_s!==null&&(!Number.isFinite(c.trigger_s)||c.trigger_s<0||c.trigger_s>=c.duration_s)))throw Error('Повреждена ссылка на испытание');
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
  if(new TextDecoder().decode(bytes.slice(0,8))!=='MDRUNP1\0'||
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
  const cache=globalThis.caches?await caches.open('alfred-microduck-runup-clips-v1').catch(()=>null):null;
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
