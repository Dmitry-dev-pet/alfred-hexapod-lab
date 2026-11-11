// Exact MuJoCo hinge/free-joint forward kinematics, without runtime physics.
export const qmul=(a,b)=>[
 a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
 a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
 a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
 a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
export function rotate(q,v){
 const x=q[1],y=q[2],z=q[3],w=q[0];
 const cx=y*v[2]-z*v[1],cy=z*v[0]-x*v[2],cz=x*v[1]-y*v[0];
 return [v[0]+2*(w*cx+y*cz-z*cy),v[1]+2*(w*cy+z*cx-x*cz),v[2]+2*(w*cz+x*cy-y*cx)];
}
export const add=(a,b)=>a.map((v,i)=>v+b[i]);
export const sub=(a,b)=>a.map((v,i)=>v-b[i]);
export function slerp(a,b,t){
 let dot=a.reduce((s,v,i)=>s+v*b[i],0);if(dot<0){b=b.map(v=>-v);dot=-dot;}
 let q;if(dot>.9995)q=a.map((v,i)=>v+(b[i]-v)*t);
 else{const angle=Math.acos(Math.min(1,dot)),s=Math.sin(angle);q=a.map((v,i)=>(Math.sin((1-t)*angle)*v+Math.sin(t*angle)*b[i])/s);}
 const norm=Math.hypot(...q);return q.map(v=>v/norm);
}
export function blendPose(a,b,t){const out=a.map((v,i)=>v+(b[i]-v)*t);out.splice(3,4,...slerp(a.slice(3,7),b.slice(3,7),t));return out;}
export function forwardKinematics(rig,qpos){
 if(qpos.length!==rig.nq||!qpos.every(Number.isFinite))throw Error('Invalid kinematic state');
 const bodies=[{p:[0,0,0],q:[1,0,0,0]}];
 for(let i=1;i<rig.bodies.length;i++){
  const b=rig.bodies[i],parent=bodies[b.parent];let p=add(parent.p,rotate(parent.q,b.pos)),q=qmul(parent.q,b.quat);
  for(const jid of b.joints){const j=rig.joints[jid];
   if(j.type===0){p=qpos.slice(j.qadr,j.qadr+3);q=qpos.slice(j.qadr+3,j.qadr+7);}
   else if(j.type===3){
    const angle=qpos[j.qadr]-j.ref,s=Math.sin(angle/2),pivot=add(p,rotate(q,j.anchor));
    q=qmul(q,[Math.cos(angle/2),...j.axis.map(v=>v*s)]);p=sub(pivot,rotate(q,j.anchor));
   }else throw Error('Unsupported joint in original rig');
  }
  bodies.push({p,q});
 }
 return rig.geoms.map(g=>{const b=bodies[g.body];return {p:add(b.p,rotate(b.q,g.pos)),q:qmul(b.q,g.quat)};});
}
