/** Two independent control groups on top of precomputed Freenove V7 trajectories.
 * Factory walking always stays active; right pad adds precomputed body pose or
 * the 2 calibrated CAD-axis previews from V5. No online IK or dynamics.
 * Result remains KINEMATIC PREVIEW (not real robot commands).
 */
import {FactoryPilot, oemBodyTargets} from '../freenove-original-v7/factory-controller.js';
const clip=(x,low=-1,high=1)=>Math.min(high,Math.max(low,x));
const bodyNames=['forward','left','height','roll','pitch','yaw'];
const rad=Math.PI/180;
const zeroBody=()=>Object.fromEntries(bodyNames.map(k=>[k,0]));
const approach=(a,b,maxDelta)=>a+clip(b-a,-maxDelta,maxDelta);

/** Left Y is travel; left X is TURN by default. Strafe needs an explicit switch,
 * because three independent motion DOF cannot fit in a two-axis thumbstick. */
export function walkingInput(stick,sideMode='turn',keyboard={forward:0,turn:0,strafe:0}){
 if(!['turn','strafe'].includes(sideMode))throw Error('Invalid left-stick mode');
 const held=stick?.pointer!==null && stick?.pointer!==undefined;
 const x=held?clip(stick.x):0, y=held?clip(stick.y):0;
 return {forward:held?y:clip(keyboard.forward),left:sideMode==='strafe'?(held?-x:clip(keyboard.strafe)):clip(keyboard.strafe),turn:sideMode==='turn'?(held?-x:clip(keyboard.turn)):clip(keyboard.turn)};
}

/** Right stick is a positioning device. Release *holds* the chosen body/head
 * target. STOP / Neutral explicitly reset it, while walking remains independent. */
export function rightInput(stick,mode){
 if(!['tilt','shift','head'].includes(mode))throw Error('Invalid right-stick mode');
 if(stick?.pointer===null||stick?.pointer===undefined)return null;
 const x=clip(stick.x),y=clip(stick.y);
 if(mode==='tilt')return {body:{roll:x,pitch:y}};
 if(mode==='shift')return {body:{forward:y,left:-x}};
 return {head:{yaw:x,pitch:y}};
}

export class CombinedPilot {
 constructor(factory,reference,headKit){
  this.factory=factory;
  if(headKit?.format!=='fnk0052-cad30-offline-motions-v5'||headKit?.head_joint_indices?.yaw!==18||headKit?.head_joint_indices?.pitch!==19||headKit?.base_kit_sha256!==factory.source.v4_ref_sha256)throw Error('Missing or incompatible head CAD30 kit');
  this.headKit=headKit;
  this.walk=new FactoryPilot(factory,reference); // always walk mode
  this.bodyTarget=zeroBody();this.body=zeroBody();this.headTarget={yaw:0,pitch:0};this.head={yaw:0,pitch:0};
 }
 setGait(value){this.walk.setGait(value);}
 setTempo(value){this.walk.setTempo(value);}
 drive(forward=0,left=0,turn=0){this.walk.command(forward,left,turn);}
 setBody(key,value){if(!bodyNames.includes(key)||!Number.isFinite(value))throw Error('Bad body axis');this.bodyTarget[key]=clip(value);}
 setHead(key,value){if(!['yaw','pitch'].includes(key)||!Number.isFinite(value))throw Error('Bad head axis');this.headTarget[key]=clip(value);}
 setRight(command){if(!command)return;for(const [key,val] of Object.entries(command.body||{}))this.setBody(key,val);for(const [key,val] of Object.entries(command.head||{}))this.setHead(key,val);}
 stop(){this.walk.stop();this.bodyTarget=zeroBody();this.headTarget={yaw:0,pitch:0};}
 resetPose(){this.bodyTarget=zeroBody();this.headTarget={yaw:0,pitch:0};}
 tick(dt){
  const state=this.walk.tick(dt),max=Math.max(0,dt)*1.8;
  for(const key of bodyNames)this.body[key]=approach(this.body[key],this.bodyTarget[key],max);
  for(const key of ['yaw','pitch'])this.head[key]=approach(this.head[key],this.headTarget[key],max);
  const k=this.factory,bq=oemBodyTargets(k,this.body),q=state.q.slice(),soft=k.cad_adapter.joint_ik_bound_rad;
  // Preserve walking leg targets first; proportionally reduce impossible mixed
  // body deltas rather than separately clipping individual legs arbitrarily.
  let bodyGain=1;
  for(let i=0;i<18;i++){
   const delta=bq[i]-k.q_home[i];
   if(delta>0&&q[i]+delta>soft)bodyGain=Math.min(bodyGain,Math.max(0,(soft-q[i])/delta));
   if(delta<0&&q[i]+delta< -soft)bodyGain=Math.min(bodyGain,Math.max(0,(-soft-q[i])/delta));
  }
  bodyGain=clip(bodyGain,0,1);
  for(let i=0;i<18;i++)q[i]=clip(q[i]+bodyGain*(bq[i]-k.q_home[i]),-soft,soft);
  for(const axis of ['yaw','pitch']){
   const id=this.headKit.head_joint_indices[axis];
   const sign=this.headKit.head_joint_signs[axis];
   const limit=this.headKit.head_limits[axis+'_rad'];
   q[id]=clip(k.q_home[id]+sign*this.head[axis]*limit,-.7,.7);
  }
  const f=k.factory_commands,b=this.body;
  const offset=[-b.left*f.position_xy_clamp_mm/1000,b.forward*f.position_xy_clamp_mm/1000,b.height*f.position_z_clamp_mm/1000].map(v=>v*bodyGain);
  const attitude=f.attitude_clamp_deg*rad;
  const rot=[-b.pitch*attitude,b.roll*attitude,b.yaw*attitude].map(v=>v*bodyGain);
  return {...state,q,rootOffsetWorld:offset,bodyRotvecWorld:rot,body:{...b},head:{...this.head},bodyGain,combinedPreview:true,kinematic:true,clamped:state.clamped||bodyGain<.999};
 }
}
