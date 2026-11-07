/** Ground-referenced kinematic preview, not a contact force or physics solver.
 * Fit walking planar position/orientation to feet currently low enough for stance.
 * A newly landing foot receives a world anchor, lifted feet release their anchor.
 */
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const finite=x=>Number.isFinite(x);
export function transformFoot(p,pose){
 const c=Math.cos(pose.yaw),s=Math.sin(pose.yaw);
 return {x:c*p.x-s*p.y+pose.x,y:s*p.x+c*p.y+pose.y,z:p.z};
}
export function rigidFitXY(pairs){
 if(pairs.length<2)return null;
 let px=0,py=0,ax=0,ay=0;
 for(const {local,anchor} of pairs){px+=local.x;py+=local.y;ax+=anchor.x;ay+=anchor.y;}
 const n=pairs.length;px/=n;py/=n;ax/=n;ay/=n;
 let dot=0,cross=0;
 for(const {local,anchor} of pairs){
  const lx=local.x-px,ly=local.y-py,wx=anchor.x-ax,wy=anchor.y-ay;
  dot+=lx*wx+ly*wy;cross+=lx*wy-ly*wx;
 }
 if(Math.hypot(dot,cross)<1e-12)return null;
 const yaw=Math.atan2(cross,dot),c=Math.cos(yaw),s=Math.sin(yaw);
 return {x:ax-c*px+s*py,y:ay-s*px-c*py,yaw};
}
export class GroundLock {
 constructor(kit,{floorThresholdM=.003}={}){
  if(kit?.format!=='fnk0052-cad30-planted-yaw-v11'||Object.keys(kit.cad_tip_local||{}).length!==6)throw Error('Missing CAD30 contact tip metadata');
  this.names=Object.keys(kit.cad_tip_local);this.threshold=floorThresholdM;
  this.anchors=new Map();this.pose={x:0,y:0,yaw:0};this.floorZ=0;this.initialized=false;
 }
 reset(toes){
  this.pose={x:0,y:0,yaw:0};this.anchors.clear();
  for(const name of this.names){const p=toes[name];if(!p||!['x','y','z'].every(k=>finite(p[k])))throw Error('Missing FK toe '+name);
   this.anchors.set(name,{x:p.x,y:p.y});}
  this.initialized=true;
  return {pose:{...this.pose},stanceCount:this.names.length,maxSlipMm:0,kinematic:true};
 }
 step(toes){
  if(!this.initialized)return this.reset(toes);
  for(const name of this.names){
   const p=toes[name];if(!p||!['x','y','z'].every(k=>finite(p[k])))throw Error('Non-finite toe '+name);
   if(p.z>this.floorZ+this.threshold){this.anchors.delete(name);continue;}
   if(!this.anchors.has(name)){
    const loc=transformFoot(p,this.pose);this.anchors.set(name,{x:loc.x,y:loc.y});
   }
  }
  const pairs=[...this.anchors].map(([name,anchor])=>({name,local:toes[name],anchor}));
  const fitted=rigidFitXY(pairs);
  if(fitted){
   let yaw=fitted.yaw;
   while(yaw-this.pose.yaw>Math.PI)yaw-=2*Math.PI;
   while(yaw-this.pose.y< -Math.PI)yaw+=2*Math.PI;
   this.pose={x:fitted.x,y:fitted.y,yaw};
  }
  const errors=pairs.map(x=>{const now=transformFoot(x.local,this.pose);return Math.hypot(now.x-x.anchor.x,now.y-x.anchor.y)*1000;});
  return {pose:{...this.pose},stanceCount:pairs.length,maxSlipMm:errors.length?Math.max(...errors):null,kinematic:true};
 }
}
