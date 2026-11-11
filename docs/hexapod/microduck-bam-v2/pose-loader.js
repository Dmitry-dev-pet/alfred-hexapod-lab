// Compact source-bound FK tracks; one on-demand download per selected clip.
const PREFIX='/assets/robot-replays/microduck-bam-v2/';
const DIGEST=/^[0-9a-f]{64}$/;
const MODES=['xml_pd','bam','bam_backlash'];
const CASES=['stand','sit','fold','roll180','pitch180','side90','push'];
const SEEDS=[20261109,20261110,20261111];
const PIN='bae39f5900b318f7e87fc485c30a31213cfa14eb';
export function validateIndex(doc, geometryDigest) {
  if(doc?.schema!=='microduck-bam-replays/v2'||doc.model_commit!==PIN||
     doc.model_package_sha256!==geometryDigest||!DIGEST.test(doc.source_summary_sha256)||
     doc.frame_count!==300||doc.duration_s!==6||doc.frame_stride!==498||
     doc.visible_mesh_count!==70||!Array.isArray(doc.clips)||doc.clips.length!==63)
    throw Error('Некорректный каталог физических записей');
  const seen=new Set();
  for(const clip of doc.clips){
    const key=[clip.mode,clip.case,clip.seed].join(':');
    const name=clip.mode+'-'+clip.case+'-'+clip.seed+'-'+String(clip.sha256).slice(0,12)+'.bin';
    if(!MODES.includes(clip.mode)||!CASES.includes(clip.case)||!SEEDS.includes(clip.seed)||
       seen.has(key)||!DIGEST.test(clip.sha256)||!DIGEST.test(clip.trace_sha256)||
       !DIGEST.test(clip.scene_sha256)||clip.url!==PREFIX+name||
       clip.size!==16+300*498*4||!['PASS','FAIL'].includes(clip.status)||
       !Number.isFinite(clip.final_displacement_m)||!Number.isFinite(clip.peak_floor_resultant_force_n))
      throw Error('Повреждена ссылка на физическую запись');
    seen.add(key);
  }
  return doc;
}
export async function decodePoseBytes(bytes,clip,index){
  if(bytes.byteLength!==clip.size||bytes.byteLength!==16+index.frame_count*index.frame_stride*4)
    throw Error('Неверный размер физической записи');
  const digest=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),
    b=>b.toString(16).padStart(2,'0')).join('');
  if(digest!==clip.sha256)throw Error('Контрольная сумма физической записи не совпадает');
  const head=new DataView(bytes);
  if(new TextDecoder().decode(bytes.slice(0,8))!=='MDPOSE2\0'||
     head.getUint32(8,true)!==300||head.getUint32(12,true)!==498)
    throw Error('Неподдерживаемый формат физической записи');
  const floats=new Float32Array(300*498);
  for(let i=0;i<floats.length;i++)floats[i]=head.getFloat32(16+i*4,true);
  const frames=[];let previous=0;
  for(let n=0;n<300;n++){
    const f=floats.subarray(n*498,(n+1)*498);
    if(!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён физический кадр');
    for(let j=0;j<70;j++){
      const k=8+j*7+3,norm=Math.hypot(f[k],f[k+1],f[k+2],f[k+3]);
      if(Math.abs(norm-1)>.001)throw Error('Повреждён поворот детали');
    }
    previous=f[0];frames.push(f);
  }
  if(previous!==6)throw Error('Неверная длительность записи');
  return {...clip,duration_s:6,frame_count:300,frames,gate_stable_at_end:clip.stable_at_end_1s};
}
export async function loadPoseClip(clip,index){
  const cache=globalThis.caches?await caches.open('alfred-microduck-pose-clips-v2').catch(()=>null):null;
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
