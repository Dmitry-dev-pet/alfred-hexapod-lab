/** OEM walking + offline exact planted body yaw. No real-time IK or physics. */
import {CombinedPilot} from '../freenove-combined-v9/combo-controller.js';
import {oemBodyTargets} from '../freenove-original-v7/factory-controller.js';
const clip=(x,a,b)=>Math.min(b,Math.max(a,x));
const rad=Math.PI/180;
export function plantedYawRow(kit,degrees){
 if(kit?.format!=='fnk0052-cad30-planted-yaw-v11')throw Error('Wrong planted-yaw kit');
 const x=clip(degrees,-15,15),lo=Math.floor(x),hi=Math.ceil(x),t=x-lo;
 const a=kit.rows[String(lo)],b=kit.rows[String(hi)];
 if(!a||!b||a.length!==20||b.length!==20)throw Error('Planted yaw row missing');
 return a.map((v,i)=>v+(b[i]-v)*t);
}
export class PlantedCombinedPilot extends CombinedPilot {
 constructor(factory,reference,headKit,plantedKit){
  super(factory,reference,headKit);
  if(plantedKit?.format!=='fnk0052-cad30-planted-yaw-v11'||plantedKit.values_degrees?.length!==31||plantedKit.model_sha256!==factory.source.model_sha256)throw Error('Invalid planted-body kit/model');
  this.planted=plantedKit;
 }
 tick(dt){
  const s=super.tick(dt),k=this.factory,b=this.body;
  const yawDegrees=b.yaw*k.factory_commands.attitude_clamp_deg;
  const yawPose=plantedYawRow(this.planted,yawDegrees);
  const bodyWithoutYaw={...b,yaw:0};
  const bodyQ=oemBodyTargets(k,bodyWithoutYaw);
  const source=this.walk.displayQ; // exact OEM walking targets before body mixing
  const soft=k.cad_adapter.joint_ik_bound_rad;
  let gain=1;
  for(let i=0;i<18;i++){
   const delta=(bodyQ[i]-k.q_home[i])+(yawPose[i]-k.q_home[i]);
   if(delta>0&&source[i]+delta>soft)gain=Math.min(gain,Math.max(0,(soft-source[i])/delta));
   if(delta<0&&source[i]+delta< -soft)gain=Math.min(gain,Math.max(0,(-soft-source[i])/delta));
  }
  gain=clip(gain,0,1);
  for(let i=0;i<18;i++)s.q[i]=clip(source[i]+gain*((bodyQ[i]-k.q_home[i])+(yawPose[i]-k.q_home[i])),-soft,soft);
  const f=k.factory_commands;
  s.rootOffsetWorld=[-b.left*f.position_xy_clamp_mm/1000,b.forward*f.position_xy_clamp_mm/1000,b.height*f.position_z_clamp_mm/1000].map(v=>v*gain);
  s.bodyRotvecWorld=[-b.pitch*f.attitude_clamp_deg*rad,b.roll*f.attitude_clamp_deg*rad,yawDegrees*rad].map(v=>v*gain);
  s.bodyGain=gain;s.clamped=s.clamped||gain<.999;
  s.plantedYaw=true;
  return s;
 }
}
