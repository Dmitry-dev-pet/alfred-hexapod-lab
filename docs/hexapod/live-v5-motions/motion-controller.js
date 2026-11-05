/** CAD30 multi-mode joystick, purely offline joint targets and FK data.
 * Gait engine from V4 is preserved; no MuJoCo/WebSocket/ROS/PWM at runtime.
 */
import {Pilot as WalkingPilot,GAITS,STRIDES} from '../live-v4-gaits/kinematic-controller.js';
export const MODES={walk:'Ходьба',body:'Корпус',leg:'Нога',head:'Голова'};
const BODY_AXES=['forward','left','height','roll','pitch','yaw'];
const LEG_AXES=['forward','left','lift'];
const clamp=(x,min,max)=>Math.min(max,Math.max(min,x));
const zeros=(names)=>Object.fromEntries(names.map(k=>[k,0]));
const scalar=(x)=>Number.isFinite(x)?clamp(x,-1,1):0;
const blend=(a,b,k)=>a+(b-a)*k;
const lerpRate=(a,b,rate,dt)=>a+clamp(b-a,-rate*dt,rate*dt);
export function validateMotionKit(k,g){
 if(!k||k.format!=='fnk0052-cad30-offline-motions-v5'||k.study!=='issue62-body-leg-head-motions'||k.no_realtime_physics!==true||k.physical_guarantee!==false)throw Error('Неверный комплект движений');
 if(!g||g.format!=='fnk0052-cad30-kinematic-gaits-v4'||g.q_stand?.length!==20||g.skeleton?.length!==46)throw Error('Не найден базовый CAD30');
 for(const [axis,key] of BODY_AXES.map(x=>[x, x==='height'?'height_m':x==='forward'?'forward_m':x==='left'?'left_m':`${x}_rad`])){
  if(!Number.isFinite(k.pose_limits[key])||k.pose_limits[key]<=0)throw Error('Нет границы движения '+key);
  for(const side of ['pos','neg']){
   const a=k.pose_axes?.[key]?.[side];
   if(!Array.isArray(a)||a.length!==20||a.some(v=>!Number.isFinite(v)))throw Error('Неверная поза '+key+'/'+side);
  }
 }
 for(const leg of g.leg_order){
  const idx=k.leg_joint_indices?.[leg];
  if(!Array.isArray(idx)||idx.length!==3||idx.some(i=>!Number.isInteger(i)||i<0||i>=18))throw Error('Неизвестная нога '+leg);
  for(const axis of LEG_AXES){const key=axis+'_m';
   if(!Number.isFinite(k.leg_limits[key])||k.leg_limits[key]<=0)throw Error('Нет предела ноги '+axis);
   for(const side of ['pos','neg']){
    const q=k.leg_axes?.[leg]?.[key]?.[side];
    if(!Array.isArray(q)||q.length!==3||q.some(v=>!Number.isFinite(v)))throw Error('Неверная таблица ноги '+leg+'/'+key);
   }
  }
 }
 if(k.head_joint_indices?.yaw!==18||k.head_joint_indices?.pitch!==19)throw Error('Неверный порядок головы');
 return true;
}
export function normalizeAxes(input,maxSum=1.3){
 const a=Object.fromEntries(Object.entries(input).map(([key,x])=>[key,scalar(x)]));
 const total=Object.values(a).reduce((sum,x)=>sum+Math.abs(x),0);
 const ratio=Math.max(1,total/maxSum);
 for(const key in a)a[key]/=ratio;
 return a;
}
export function poseJointTargets(g,k,body,legIndex,leg){
 const q=g.q_stand.slice();
 const axes=normalizeAxes(body,1.3);
 for(const key of BODY_AXES){
  const value=axes[key];if(Math.abs(value)<1e-8)continue;
  const name=(key==='forward'||key==='left'||key==='height')?key+'_m':key+'_rad';
  const base=k.pose_axes[name][value>=0?'pos':'neg'];
  for(let j=0;j<18;j++)q[j]+=Math.abs(value)*(base[j]-g.q_stand[j]);
 }
 if(legIndex && g.leg_order.includes(legIndex)){
  const foot=normalizeAxes(leg,1.3);
  const indices=k.leg_joint_indices[legIndex];
  for(const key of LEG_AXES){
   const value=foot[key];if(Math.abs(value)<1e-8)continue;
   const source=k.leg_axes[legIndex][key+'_m'][value>=0?'pos':'neg'];
   for(let j=0;j<3;j++)q[indices[j]]+=Math.abs(value)*(source[j]-g.q_stand[indices[j]]);
  }
 }
 return q.map((x,j)=>j<18?clamp(x,-1.7,1.7):x);
}
export class MotionPilot{
 constructor(gaitKit,motionKit){
  validateMotionKit(motionKit,gaitKit);
  this.base=gaitKit;this.ext=motionKit;this.walk=new WalkingPilot(gaitKit);
  this.mode='walk';this.activeLeg=gaitKit.leg_order[0];
  this.bodyTarget=zeros(BODY_AXES);this.bodyNow=zeros(BODY_AXES);
  this.footTarget=zeros(LEG_AXES);this.footNow=zeros(LEG_AXES);
  this.headTarget={yaw:0,pitch:0};this.headNow={yaw:0,pitch:0};
  this.displayQ=gaitKit.q_stand.slice();
 }
 setMode(mode){
  if(!(mode in MODES))throw Error('Неизвестный режим');
  if(mode===this.mode)return;
  this.stopAll();this.mode=mode;
 }
 setGait(n){this.walk.setGait(n);}
 setStride(n){this.walk.setStride(n);}
 setTempo(n){this.walk.setTempo(n);}
 setLeg(code){if(!this.base.leg_order.includes(code))throw Error('Неизвестная нога');this.activeLeg=code;this.footTarget=zeros(LEG_AXES);}
 setBody(axis,value){if(!BODY_AXES.includes(axis)||!Number.isFinite(value))throw Error('Неверная поза');this.bodyTarget[axis]=scalar(value);}
 setFoot(axis,value){if(!LEG_AXES.includes(axis)||!Number.isFinite(value))throw Error('Неверная команда ноги');this.footTarget[axis]=scalar(value);}
 setHead(axis,value){if(!['yaw','pitch'].includes(axis)||!Number.isFinite(value))throw Error('Неверная команда головы');this.headTarget[axis]=scalar(value);}
 control(stickForward,stickLeft,turn=0){
  if(![stickForward,stickLeft,turn].every(Number.isFinite))throw Error('Неверная команда джойстика');
  if(this.mode==='walk')this.walk.command(stickForward,stickLeft,turn);
  else if(this.mode==='body'){
   this.setBody('forward',stickForward);this.setBody('left',stickLeft);
  }else if(this.mode==='leg'){
   this.setFoot('forward',stickForward);this.setFoot('left',stickLeft);
  }else if(this.mode==='head'){
   this.setHead('yaw',stickLeft);this.setHead('pitch',stickForward);
  }
 }
 stopAll(){
  this.walk.stop();this.bodyTarget=zeros(BODY_AXES);this.footTarget=zeros(LEG_AXES);this.headTarget={yaw:0,pitch:0};
 }
 reset(){
  this.walk.reset();this.bodyTarget=zeros(BODY_AXES);this.bodyNow=zeros(BODY_AXES);
  this.footTarget=zeros(LEG_AXES);this.footNow=zeros(LEG_AXES);
  this.headTarget={yaw:0,pitch:0};this.headNow={yaw:0,pitch:0};this.displayQ=this.base.q_stand.slice();
 }
 tick(dt){
  if(!Number.isFinite(dt)||dt<0||dt>.1)throw Error('Контроллер dt вне диапазона');
  const w=this.walk.tick(dt);
  const step=(curr,target,rate)=>{for(const key in curr)curr[key]=lerpRate(curr[key],target[key],rate,dt);};
  step(this.bodyNow,this.bodyTarget,1.6);
  step(this.footNow,this.footTarget,1.6);
  step(this.headNow,this.headTarget,1.8);
  const body=normalizeAxes(this.bodyNow,1.3);
  const foot=normalizeAxes(this.footNow,1.3);
  const qPost=poseJointTargets(this.base,this.ext,body,this.activeLeg,foot);
  const q=w.q.map((v,j)=>clamp(v+(qPost[j]-this.base.q_stand[j]),-1.7,1.7));
  q[18]=clamp(this.base.q_stand[18]+this.ext.head_joint_signs.yaw*this.headNow.yaw*this.ext.head_limits.yaw_rad,-.7,.7);
  q[19]=clamp(this.base.q_stand[19]+this.ext.head_joint_signs.pitch*this.headNow.pitch*this.ext.head_limits.pitch_rad,-.7,.7);
  const maxAngleMove=Math.max(3.5,Math.min(8,2*this.walk.tempo))*dt;
  this.displayQ=q.map((v,j)=>lerpRate(this.displayQ[j],v,maxAngleMove,dt));
  const L=this.ext.pose_limits, f=this.base.forward_world,l=this.base.left_world;
  const shift=[0,0,body.height*L.height_m];
  for(let i=0;i<3;i++)shift[i]+=f[i]*body.forward*L.forward_m+l[i]*body.left*L.left_m;
  const rotation=[0,0,body.yaw*L.yaw_rad];
  for(let i=0;i<3;i++)rotation[i]+=f[i]*body.roll*L.roll_rad+l[i]*body.pitch*L.pitch_rad;
  return {...w,q:this.displayQ.slice(),body:{...body},foot:{...foot},head:{...this.headNow},rootOffsetWorld:shift,bodyRotvecWorld:rotation,mode:this.mode,leg:this.activeLeg,kinematic:true};
 }
}
