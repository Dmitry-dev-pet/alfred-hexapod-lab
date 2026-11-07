/** Freenove Original amplitude adaptation for the existing Hexapod Lab.
 * Source trace: Freenove control.py (run_gait + CMD_POSITION + CMD_ATTITUDE).
 * The manufacturer gait phases are OFFLINE converted to CAD30 qpos tables.
 * Browser performs linear interpolation & FK only; never physical dynamics or PWM.
 */
export const GAITS={tripod:'Tripod (Freenove 1)',wave:'Wave (Freenove 2)'};
export const BODY_AXES=['forward','left','height','roll','pitch','yaw'];
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const BODY_NAMES={forward:'forward_m',left:'left_m',height:'height_m',roll:'roll_rad',pitch:'pitch_rad',yaw:'yaw_rad'};
const DIRECTIONS=['forward','forward_left','left','back_left','back','back_right','right','forward_right'];
const AXES=()=>({forward:0,left:0,height:0,roll:0,pitch:0,yaw:0});
function row(table,phase){
 const a=(((phase%1)+1)%1)*table.length,i=Math.floor(a),next=(i+1)%table.length,f=a-i;
 return table[i].map((x,j)=>x+(table[next][j]-x)*f);
}
export function validateFactoryKit(kit,ref){
 if(kit?.format!=='fnk0052-factory-cad30-kinematic-v7'||ref?.format!=='fnk0052-cad30-kinematic-gaits-v4')throw Error('Таблицы Freenove Original не подходят');
 if(kit.joint_count!==20||kit.phase_samples!==48||kit.q_home?.length!==20||ref.q_stand?.length!==20||ref.skeleton?.length!==46)throw Error('Повреждённый CAD-риг');
 for(const gait of Object.keys(GAITS)){
  const g=kit.gaits?.[gait];if(!g||!(g.cycle_s>.25&&g.cycle_s<2))throw Error('Отсутствует заводская походка '+gait);
  for(const key of [...DIRECTIONS,'ccw','cw','lift']){
   const rows=g.rows?.[key];if(!Array.isArray(rows)||rows.length!==48||rows.some(r=>r.length!==20||r.some(x=>!Number.isFinite(x))))throw Error('Нет массива '+gait+'/'+key);
  }
 }
 for(const key of BODY_AXES){
  for(const side of ['neg','pos']){
   const q=kit.body_axes?.[BODY_NAMES[key]]?.[side];if(!Array.isArray(q)||q.length!==20||q.some(x=>!Number.isFinite(x)))throw Error('Нет позы '+key+side);
  }
 }
 if(kit.factory_commands?.move_xy_clamp_mm!==35||kit.factory_commands?.lift_mm!==40||kit.factory_commands?.position_xy_clamp_mm!==40||kit.factory_commands?.position_z_clamp_mm!==20||kit.factory_commands?.attitude_clamp_deg!==15)throw Error('Не заводские параметры Freenove');
 if(kit.cad_adapter?.unverified_real_motor_bounds!==true)throw Error('Утрачен статус калибровки');
 return true;
}
function angularRow(g,phase,forward,left){
 const angle=(Math.atan2(left,forward)+Math.PI*2)%(Math.PI*2),pos=angle/(Math.PI/4),index=Math.floor(pos)%8,frac=pos-Math.floor(pos);
 const a=row(g.rows[DIRECTIONS[index]],phase),b=row(g.rows[DIRECTIONS[(index+1)%8]],phase);
 return a.map((x,i)=>x+(b[i]-x)*frac);
}
export function oemWalkingTargets(k,gait,phase,forward,left,turn){
 const g=k.gaits[gait];if(!g)throw Error('Неизвестная заводская походка');
 const mag=clamp(Math.hypot(forward,left),0,1),yaw=clamp(turn,-1,1);
 if(mag<1e-8&&Math.abs(yaw)<1e-8)return k.q_home.slice();
 const lift=row(g.rows.lift,phase);
 const a=mag>1e-8?angularRow(g,phase,forward,left):lift;
 const b=row(g.rows[yaw>=0?'ccw':'cw'],phase);
 return lift.map((v,i)=>v+mag*(a[i]-v)+Math.abs(yaw)*(b[i]-v));
}
export function oemBodyTargets(k,body){
 const q=k.q_home.slice();
 for(const key of BODY_AXES){
  const x=clamp(Number(body[key])||0,-1,1);if(Math.abs(x)<1e-9)continue;
  const a=k.body_axes[BODY_NAMES[key]][x>=0?'pos':'neg'];
  for(let j=0;j<18;j++)q[j]+=Math.abs(x)*(a[j]-k.q_home[j]);
 }
 return q;
}
export class FactoryPilot{
 constructor(factory,ref){validateFactoryKit(factory,ref);this.k=factory;this.ref=ref;this.mode='walk';this.gait='tripod';this.tempo=1;this.phase=0;this.x=0;this.y=0;this.yaw=0;this.target={forward:0,left:0,turn:0};this.now={...this.target};this.bodyTarget=AXES();this.body=AXES();this.displayQ=factory.q_home.slice();this.totalClamps=0;}
 setMode(mode){if(!['walk','body'].includes(mode))throw Error('Неизвестный режим');if(mode!==this.mode){this.stop();this.mode=mode;this.phase=0;}}
 setGait(gait){if(!(gait in GAITS))throw Error('Нет такой заводской походки');this.gait=gait;}
 setTempo(value){if(!Number.isFinite(value)||value<.25||value>2)throw Error('Неверный темп');this.tempo=value;}
 command(forward=0,left=0,turn=0){if(![forward,left,turn].every(Number.isFinite))throw Error('Некорректная команда');const r=Math.hypot(forward,left),factor=Math.max(1,r);this.target={forward:clamp(forward/factor,-1,1),left:clamp(left/factor,-1,1),turn:clamp(turn,-1,1)};}
 setBody(key,value){if(!BODY_AXES.includes(key)||!Number.isFinite(value))throw Error('Некорректное движение корпуса');this.bodyTarget[key]=clamp(value,-1,1);}
 stop(){this.target={forward:0,left:0,turn:0};this.bodyTarget=AXES();}
 reset(){this.stop();this.phase=0;this.now={...this.target};this.body=AXES();this.x=this.y=this.yaw=0;this.displayQ=this.k.q_home.slice();}
 tick(dt){
  if(!Number.isFinite(dt)||dt<0||dt>.1)throw Error('Время кадра за пределами');
  const smooth=(a,b,speed)=>a+clamp(b-a,-speed*dt,speed*dt);
  for(const key of Object.keys(this.now))this.now[key]=smooth(this.now[key],this.mode==='walk'?this.target[key]:0,3.4);
  for(const key of BODY_AXES)this.body[key]=smooth(this.body[key],this.mode==='body'?this.bodyTarget[key]:0,1.8);
  const g=this.k.gaits[this.gait],amp=Math.max(Math.hypot(this.now.forward,this.now.left),Math.abs(this.now.turn));
  if(this.mode==='walk'&&amp>1e-6)this.phase=(this.phase+dt*this.tempo/g.cycle_s)%1;
  const target=this.mode==='walk'?oemWalkingTargets(this.k,this.gait,this.phase,this.now.forward,this.now.left,this.now.turn):oemBodyTargets(this.k,this.body);
  const rawMin=Math.min(...target.slice(0,18)),rawMax=Math.max(...target.slice(0,18));
  const soft=this.k.cad_adapter.joint_ik_bound_rad;
  const clamped=rawMin< -soft||rawMax>soft;
  if(clamped)this.totalClamps++;
  const maxMoveRate=5.2*this.tempo;
  this.displayQ=target.map((v,i)=>{
   const x=i<18?clamp(v,-soft,soft):v;
   return smooth(this.displayQ[i],x,maxMoveRate);
  });
  if(this.mode==='walk'){
   const step=g.factory_full_stroke_mm/1000,progress=step/g.cycle_s*this.tempo;
   const f=this.now.forward*progress*dt,l=this.now.left*progress*dt,c=Math.cos(this.yaw),s=Math.sin(this.yaw);
   this.x+=-s*f-c*l;this.y+=c*f-s*l;
   this.yaw+=this.now.turn*(this.k.factory_commands.vendor_angle_deg_profile*Math.PI/180)/g.cycle_s*this.tempo*dt;
  }
  const b=this.body,cm=this.k.factory_commands;
  const rootOffsetWorld=[-b.left*cm.position_xy_clamp_mm/1000,b.forward*cm.position_xy_clamp_mm/1000,b.height*cm.position_z_clamp_mm/1000];
  const attitude=cm.attitude_clamp_deg*Math.PI/180;
  const bodyRotvecWorld=[-b.pitch*attitude,b.roll*attitude,b.yaw*attitude];
  return {q:this.displayQ.slice(),phase:this.phase,mode:this.mode,gait:this.gait,command:{...this.now},body:{...this.body},x:this.x,y:this.y,yaw:this.yaw,rootOffsetWorld,bodyRotvecWorld,moving:amp>1e-5,clamped,tempo:this.tempo,kinematic:true};
 }
}
