export const SOURCE_COMMIT = '7318411cfbfbffdbd68f75e3734a50c3933969e5';
export async function sha256(bytes) {
  return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)),b=>b.toString(16).padStart(2,'0')).join('');
}
const finite = (a,n) => Array.isArray(a)&&a.length===n&&a.every(Number.isFinite);
export function validatePackage(p) {
  if(p?.schema!=='alfred-bb8-player/v1'||p.source_commit!==SOURCE_COMMIT||
    Object.keys(p.meshes||{}).length!==26||p.instances?.length!==54||p.replay_instances?.length!==42||p.traces?.length!==3)
    throw Error('Неверный пакет исходной сборки BB‑8');
  for(const [key,m] of Object.entries(p.meshes)) {
    if(!/^part-\d\d$/.test(key)||!Number.isInteger(m.size)||m.size<84||m.size>1_000_000||
      typeof m.stl!=='string'||m.stl.length>1_400_000||!/^[0-9a-f]{64}$/.test(m.sha256))throw Error('Повреждена деталь сборки');
  }
  for(const i of [...p.instances,...p.replay_instances]) {
    if(!p.meshes[i.mesh]||!finite(i.position,3)||!finite(i.quaternion,4)||i.position.some(x=>Math.abs(x)>2)||
      Math.abs(i.quaternion.reduce((s,x)=>s+x*x,0)-1)>.00001)throw Error('Неверное размещение детали');
  }
  for(const t of p.traces) {
    if(t.calibrated!==false||t.source_git_commit!==SOURCE_COMMIT||t.frames?.length!==301||t.duration_s!==6)throw Error('Неверная запись движения');
    t.frames.forEach((f,index)=>{
      if(!finite(f.qpos,8)||!finite(f.ctrl,1)||Math.abs(f.t-index*.02)>1e-6)throw Error('Повреждены кадры записи');
    });
  }
  return p;
}
export function frameAt(trace,time) {
  return trace.frames[Math.max(0,Math.min(trace.frames.length-1,Math.round(time/.02)))];
}
