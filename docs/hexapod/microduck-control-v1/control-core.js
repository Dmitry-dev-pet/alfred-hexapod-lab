// Kinematic preview: blend prepared motions, integrate commanded travel,
// replay recorded falls/get-ups. This module contains no contact dynamics.
import {blendPose,qmul,rotate} from './kinematics.js';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const smooth=t=>{t=clamp(t,0,1);return t*t*(3-2*t);};
const yawQuat=y=>[Math.cos(y/2),0,0,Math.sin(y/2)];
export function sampleRows(rows,time){
 if(time<=rows[0][0])return rows[0].slice(1);
 if(time>=rows.at(-1)[0])return rows.at(-1).slice(1);
 let a=0,b=rows.length-1;while(b-a>1){const m=(a+b)>>1;if(rows[m][0]<time)a=m;else b=m;}
 return blendPose(rows[a].slice(1),rows[b].slice(1),(time-rows[a][0])/(rows[b][0]-rows[a][0]));
}
function cyclePose(c,phase){const x=((phase%1)+1)%1*c.qpos.length,i=Math.floor(x),j=(i+1)%c.qpos.length;return blendPose(c.qpos[i],c.qpos[j],x-i);}
export class KinematicPilot{
 constructor(rig,kit){
  if(rig.schema!=='microduck-kinematic-rig/v1'||kit.schema!=='microduck-kinematic-motions/v1'||rig.nq!==35||kit.nq!==35)throw Error('Original MicroDuck rig/motions mismatch');
  this.rig=rig;this.kit=kit;this.reset();
 }
 reset(){this.x=0;this.y=0;this.heading=0;this.phase=0;this.forward=0;this.turn=0;this.targetForward=0;this.targetTurn=0;this.head={x:0,y:0};this.state='standing';this.paused=false;this.action=null;this.settle=null;this.anchor=null;this.case=null;this.pose=this.kit.neutral_qpos.slice();return this.pose;}
 command(forward,turn,headX=0,headY=0){
  this.targetForward=clamp(Number(forward)||0,-1,1);this.targetTurn=clamp(Number(turn)||0,-1,1);
  this.head={x:clamp(Number(headX)||0,-1,1),y:clamp(Number(headY)||0,-1,1)};
 }
 stop(){this.targetForward=this.targetTurn=this.forward=this.turn=0;this.head={x:0,y:0};if(this.action)this.paused=true;}
 resume(){this.paused=false;}
 fall(name='side90'){
  if(this.action||this.state==='down'||!this.kit.actions[name])return false;
  this.stop();this.paused=false;
  const rows=this.kit.actions[name].fall,origin=rows[0].slice(1,3);
  this.anchor={x:this.x,y:this.y,heading:this.heading,origin};this.case=name;
  this.action={kind:'fall',time:0,blend:.35,rows,from:this.pose.slice()};this.state='falling';return true;
 }
 rise(){
  if(this.state!=='down'||this.action)return false;
  this.paused=false;this.action={kind:'rise',time:0,blend:0,rows:this.kit.actions[this.case].rise,from:this.pose.slice()};this.state='rising';return true;
 }
 anchored(q){
  const p=q.slice(),a=this.anchor,delta=rotate(yawQuat(a.heading),[q[0]-a.origin[0],q[1]-a.origin[1],0]);
  p[0]=a.x+delta[0];p[1]=a.y+delta[1];p.splice(3,4,...qmul(yawQuat(a.heading),q.slice(3,7)));return p;
 }
 tick(dt){
  dt=clamp(Number(dt)||0,0,.05);
  if(this.action){
   const a=this.action;if(!this.paused)a.time+=dt;
   if(a.time<a.blend)this.pose=blendPose(a.from,this.anchored(a.rows[0].slice(1)),smooth(a.time/a.blend));
   else this.pose=this.anchored(sampleRows(a.rows,a.time-a.blend));
   if(a.time-a.blend>=a.rows.at(-1)[0]){
    this.x=this.pose[0];this.y=this.pose[1];
    if(a.kind==='fall')this.state='down';
    else{this.state='standing';this.heading=Math.atan2(2*(this.pose[3]*this.pose[6]+this.pose[4]*this.pose[5]),1-2*(this.pose[5]**2+this.pose[6]**2));this.settle={from:this.pose.slice(),time:0};}
    this.action=null;this.paused=false;
   }
   return this.pose;
  }
  if(this.state==='down')return this.pose;
  const alpha=1-Math.exp(-dt*8);this.forward+=(this.targetForward-this.forward)*alpha;this.turn+=(this.targetTurn-this.turn)*alpha;
  const f=Math.abs(this.forward),t=Math.abs(this.turn),total=Math.max(1,f+t),fw=f/total,tw=t/total;
  const fc=this.kit.cycles[this.forward>=0?'forward':'backward'],tc=this.kit.cycles[this.turn>=0?'turn_left':'turn_right'];
  const duration=(fc.qpos.length*fc.dt*fw+tc.qpos.length*tc.dt*tw)/Math.max(.001,fw+tw);
  this.phase+=dt/Math.max(.3,duration);
  let q=this.kit.neutral_qpos.slice();q=blendPose(q,cyclePose(fc,this.phase),fw);q=blendPose(q,cyclePose(tc,this.phase),tw);
  this.heading+=this.turn*.6*dt;
  const speed=this.forward*(this.forward>=0?.12:.08);this.x+=Math.cos(this.heading)*speed*dt;this.y+=Math.sin(this.heading)*speed*dt;
  q[0]=this.x;q[1]=this.y;q.splice(3,4,...qmul(yawQuat(this.heading),q.slice(3,7)));
  const head=this.rig.servo_qpos,ranges=this.rig.servo_ranges;
  for(const [name,value] of [['head_yaw',this.head.x*.65],['head_pitch',-this.head.y*.35]]){
   const i=head[name];if(i!==undefined){const [lo,hi]=ranges[name];q[i]=clamp(q[i]+value,lo,hi);}
  }
  if(this.settle){this.settle.time+=dt;q=blendPose(this.settle.from,q,smooth(this.settle.time/.3));if(this.settle.time>=.3)this.settle=null;}
  this.pose=q;this.state=f+t>.03?'walking':'standing';return q;
 }
}
