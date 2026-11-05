/** Offline kinematic six-leg joystick controller.
 * No network, physics engine, solver, ROS or PWM access in this module.
 * Every joint command comes from offline precomputed IK tables. Body travel
 * is commanded path integration, not a measured displacement.
 */
export const AXES={vx:'fwd',vy:'left',wz:'ccw'};
const LIMITS={vx:.006,vy:.005,wz:.075};
function clamp(x,min,max){return Math.min(max,Math.max(min,x));}
export function validateKit(k){
 if(!k||k.format!=='fnk0052-cad30-kinematic-control-v1'||k.study!=='issue62-live-v2-offline'||k.samples!==48||k.q_stand?.length!==20||k.skeleton?.length!==46)throw Error('Неподдерживаемый набор кинематики');
 for(const key of ['fwd','back','left','right','ccw','cw']){
  const a=k.joint_tables?.[key];
  if(!Array.isArray(a)||a.length!==48||a.some(row=>!Array.isArray(row)||row.length!==20||row.some(v=>!Number.isFinite(v))))throw Error('Неверная таблица: '+key);
 }
 if(k.skeleton.some(n=>!Number.isInteger(n.id)||!Number.isInteger(n.parent)||n.p?.length!==3||n.q?.length!==4))throw Error('Некорректное кинематическое дерево');
 return true;
}
export function cycleRow(table,phase){
 const n=table.length,idx=(((phase%1)+1)%1)*n,a=Math.floor(idx),b=(a+1)%n,alpha=idx-a;
 return table[a].map((v,j)=>v*(1-alpha)+table[b][j]*alpha);
}
export function jointTargets(kit,phase,cmd){
 const q=kit.q_stand.slice();
 const selections=[
  [cmd.vx>=0?'fwd':'back',clamp(Math.abs(cmd.vx),0,1)],
  [cmd.vy>=0?'left':'right',clamp(Math.abs(cmd.vy),0,1)],
  [cmd.wz>=0?'ccw':'cw',clamp(Math.abs(cmd.wz),0,1)],
 ];
 for(const [mode,weight] of selections){
  if(weight<=1e-8)continue;
  const row=cycleRow(kit.joint_tables[mode],phase);
  for(let i=0;i<20;i++)q[i]+=weight*(row[i]-kit.q_stand[i]);
 }
 for(let i=0;i<18;i++)q[i]=clamp(q[i],-1.7,1.7);
 return q;
}
export class Pilot{
 constructor(kit){validateKit(kit);this.kit=kit;this.phase=0;this.elapsed=0;this.x=0;this.y=0;this.yaw=0;this.target={vx:0,vy:0,wz:0};this.current={vx:0,vy:0,wz:0};this.joints=kit.q_stand.slice();}
 command(vx=0,vy=0,wz=0){
  if(![vx,vy,wz].every(Number.isFinite))throw Error('Некорректная команда');
  this.target={vx:clamp(vx,-1,1),vy:clamp(vy,-1,1),wz:clamp(wz,-1,1)};
 }
 stop(){this.command(0,0,0);}
 reset(){this.stop();Object.assign(this.current,{vx:0,vy:0,wz:0});this.phase=0;this.elapsed=0;this.x=0;this.y=0;this.yaw=0;this.joints=this.kit.q_stand.slice();}
 tick(dt){
  if(!Number.isFinite(dt)||dt<0||dt>.1)throw Error('Контроллер dt вне диапазона');
  this.elapsed+=dt;
  // Smooth commands for a real thumb; zero input always ramps down.
  for(const key of ['vx','vy','wz']){
    const rate=key==='wz'?1.5:1.8;
    const diff=this.target[key]-this.current[key];
    this.current[key]+=clamp(diff,-rate*dt,rate*dt);
    if(Math.abs(this.current[key])<1e-7)this.current[key]=0;
  }
  const mag=Math.max(...Object.values(this.current).map(Math.abs));
  if(mag>1e-6)this.phase=(this.phase+dt/this.kit.period_s)%1;
  const target=jointTargets(this.kit,this.phase,this.current);
  const maxJointStep=1.7*dt;
  this.joints=target.map((q,i)=>this.joints[i]+clamp(q-this.joints[i],-maxJointStep,maxJointStep));
  // Predicted commanded pose only (never measured physics motion).
  const f=this.kit.forward_world,l=this.kit.left_world;
  const forward=this.current.vx*LIMITS.vx*dt,left=this.current.vy*LIMITS.vy*dt;
  const dx=f[0]*forward+l[0]*left,dy=f[1]*forward+l[1]*left;
  const c=Math.cos(this.yaw),s=Math.sin(this.yaw);
  this.x+=c*dx-s*dy;this.y+=s*dx+c*dy;
  this.yaw+=this.current.wz*LIMITS.wz*dt;
  return {q:this.joints,phase:this.phase,x:this.x,y:this.y,yaw:this.yaw,command:{...this.current},moving:mag>1e-5};
 }
}
