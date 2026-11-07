/**
 * Offline CAD30 gait selector: 3 genuine phase schedules, 15/25/35mm strides,
 * 6 precomputed command axes, joint blending and commanded body-motion preview.
 * No live dynamics, contact-force inference, servo output, network or solver.
 */
export const GAITS={wave:'Wave · 1 нога',ripple:'Ripple · 2 ноги',tripod:'Tripod · 3 ноги'};
export const STRIDES=[15,25,35];
export const TEMPO_MIN=.5,TEMPO_MAX=8;
const MODES=['fwd','back','left','right','ccw','cw'];
const clamp=(x,min,max)=>Math.max(min,Math.min(max,x));
const lerp=(a,b,t)=>a+(b-a)*t;

export function validateKit(k){
 if(!k||k.format!=='fnk0052-cad30-kinematic-gaits-v4'||k.study!=='issue62-v4-stride-gaits-offline'||k.samples!==48||k.period_s!==2.4||k.q_stand?.length!==20||k.skeleton?.length!==46)throw Error('Неверный набор походок');
 if(!STRIDES.every(n=>k.stride_mm_levels?.includes(n))||!Object.keys(GAITS).every(n=>k.gaits?.includes(n))||!Number.isFinite(k.nominal_turn_radius_m)||k.nominal_turn_radius_m<=0)throw Error('Неверные ограничения походок');
 if(!Array.isArray(k.forward_world)||k.forward_world.length!==3||!Array.isArray(k.left_world)||k.left_world.length!==3||![...k.forward_world,...k.left_world,...k.q_stand].every(Number.isFinite))throw Error('Неверная геометрия');
 for(const gait of Object.keys(GAITS))for(const step of STRIDES)for(const mode of MODES){
   const a=k.joint_tables?.[gait]?.[String(step)]?.[mode];
   if(!Array.isArray(a)||a.length!==48||a.some(row=>!Array.isArray(row)||row.length!==20||row.some(v=>!Number.isFinite(v))))throw Error('Отсутствует таблица '+gait+'/'+step+'/'+mode);
 }
 if(k.skeleton.some(n=>!Number.isInteger(n.id)||!Number.isInteger(n.parent)||n.p?.length!==3||n.q?.length!==4))throw Error('Неверное дерево суставов');
 return true;
}
export function cycleRow(table,phase){
 const n=table.length,idx=(((phase%1)+1)%1)*n,lo=Math.floor(idx),hi=(lo+1)%n,alpha=idx-lo;
 return table[lo].map((x,j)=>lerp(x,table[hi][j],alpha));
}
function strideRows(kit,gait,phase,step,mode){
 let lo=STRIDES[0],hi=STRIDES[STRIDES.length-1];
 for(let i=0;i<STRIDES.length-1;i++)if(step>=STRIDES[i]&&step<=STRIDES[i+1]){lo=STRIDES[i];hi=STRIDES[i+1];break;}
 const a=cycleRow(kit.joint_tables[gait][String(lo)][mode],phase);
 if(lo===hi)return a;
 const b=cycleRow(kit.joint_tables[gait][String(hi)][mode],phase);
 return a.map((v,j)=>lerp(v,b[j],clamp((step-lo)/(hi-lo),0,1)));
}
export function clampTwist(cmd){
 if(![cmd.vx,cmd.vy,cmd.wz].every(Number.isFinite))throw Error('Некорректная команда');
 let v={vx:clamp(cmd.vx,-1,1),vy:clamp(cmd.vy,-1,1),wz:clamp(cmd.wz,-1,1)};
 // Avoid stacking three full-amplitude joint excursions. This is a preview
 // envelope guard, not a proof of force/geometry feasibility.
 const total=Math.abs(v.vx)+Math.abs(v.vy)+Math.abs(v.wz);
 const scale=Math.max(1,total/1.6);
 for(const k of ['vx','vy','wz'])v[k]/=scale;
 return v;
}
export function jointTargets(kit,gait,phase,stride,command){
 if(!GAITS[gait])throw Error('Неизвестная походка');
 if(!Number.isFinite(stride)||stride<15||stride>35)throw Error('Шаг вне диапазона');
 const cmd=clampTwist(command);
 const q=kit.q_stand.slice();
 const selectors=[
  [cmd.vx>=0?'fwd':'back',Math.abs(cmd.vx)],
  [cmd.vy>=0?'left':'right',Math.abs(cmd.vy)],
  [cmd.wz>=0?'ccw':'cw',Math.abs(cmd.wz)],
 ];
 for(const [mode,weight] of selectors){
   if(weight<1e-8)continue;
   const row=strideRows(kit,gait,phase,stride,mode);
   for(let j=0;j<18;j++)q[j]+=weight*(row[j]-kit.q_stand[j]);
 }
 for(let j=0;j<18;j++)q[j]=clamp(q[j],-1.7,1.7);
 return q;
}
export class Pilot{
 constructor(kit){
   validateKit(kit);this.kit=kit;this.phase=0;this.elapsed=0;
   this.gait='wave';this.requestedGait='wave';this.transition=null;
   this.stride=35;this.requestedStride=35;this.tempo=4;
   this.x=0;this.y=0;this.yaw=0;
   this.target={vx:0,vy:0,wz:0};this.current={vx:0,vy:0,wz:0};this.joints=kit.q_stand.slice();
 }
 command(vx=0,vy=0,wz=0){
   if(![vx,vy,wz].every(Number.isFinite))throw Error('Неверная команда');
   this.target={vx:clamp(vx,-1,1),vy:clamp(vy,-1,1),wz:clamp(wz,-1,1)};
 }
 setGait(gait){if(!GAITS[gait])throw Error('Неверная походка');this.requestedGait=gait;}
 setStride(n){if(!STRIDES.includes(n))throw Error('Выбери шаг 15, 25 или 35 мм');this.requestedStride=n;}
 setTempo(t){if(!Number.isFinite(t)||t<TEMPO_MIN||t>TEMPO_MAX)throw Error('Темп вне диапазона');this.tempo=t;}
 stop(){this.command(0,0,0);}
 reset(){this.stop();Object.assign(this.current,{vx:0,vy:0,wz:0});this.phase=0;this.elapsed=0;this.x=this.y=this.yaw=0;this.transition=null;this.gait=this.requestedGait;this.stride=this.requestedStride;this.joints=this.kit.q_stand.slice();}
 tick(dt){
   if(!Number.isFinite(dt)||dt<0||dt>.1)throw Error('Контроллер dt вне диапазона');
   this.elapsed+=dt;
   for(const key of ['vx','vy','wz']){
     const diff=this.target[key]-this.current[key];
     this.current[key]+=clamp(diff,-(key==='wz'?1.5:1.8)*dt,(key==='wz'?1.5:1.8)*dt);
     if(Math.abs(this.current[key])<1e-7)this.current[key]=0;
   }
   this.stride+=clamp(this.requestedStride-this.stride,-35*dt,35*dt);
   const cmd=clampTwist(this.current);
   const mag=Math.max(Math.abs(cmd.vx),Math.abs(cmd.vy),Math.abs(cmd.wz));
   if(mag<1e-6&&!this.transition){
      this.gait=this.requestedGait;
   }
   if(mag>1e-6){
      const advance=dt*this.tempo/this.kit.period_s;
      const next=this.phase+advance;
      this.phase=next%1;
      // Phase-locked gait switch: wait for next complete cycle, then blend.
      if(next>=1&&this.gait!==this.requestedGait&&!this.transition){
         this.transition={from:this.gait,to:this.requestedGait,progress:0};
      }
   }
   let target=jointTargets(this.kit,this.gait,this.phase,this.stride,cmd);
   if(this.transition){
      this.transition.progress=clamp(this.transition.progress+dt/0.42,0,1);
      const z=this.transition.progress;const w=z*z*(3-2*z);
      const b=jointTargets(this.kit,this.transition.to,this.phase,this.stride,cmd);
      target=target.map((a,i)=>lerp(a,b[i],w));
      if(z>=1){this.gait=this.transition.to;this.transition=null;}
   }
   const maxJointStep=Math.min(5.5,this.tempo*1.7)*dt;
   this.joints=target.map((q,j)=>lerp(this.joints[j],q,Math.min(1,maxJointStep/(Math.abs(q-this.joints[j])||1))));
   // Commanded body displacement only, never actual MuJoCo or measured COM.
   const distance=this.stride/1000*this.tempo/this.kit.period_s*dt;
   const forward=cmd.vx*distance,left=cmd.vy*distance;
   const f=this.kit.forward_world,l=this.kit.left_world;
   const dx=f[0]*forward+l[0]*left,dy=f[1]*forward+l[1]*left;
   const c=Math.cos(this.yaw),s=Math.sin(this.yaw);
   this.x+=c*dx-s*dy;this.y+=s*dx+c*dy;
   this.yaw+=cmd.wz*(this.stride/1000/this.kit.nominal_turn_radius_m)*this.tempo/this.kit.period_s*dt;
   return {q:this.joints,phase:this.phase,gait:this.gait,pendingGait:this.requestedGait,strideMm:this.stride,x:this.x,y:this.y,yaw:this.yaw,command:cmd,moving:mag>1e-5,tempo:this.tempo};
 }
}
