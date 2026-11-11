/** Offline viewing of *measured* MuJoCo qpos frames; NO browser physics/IK/root welding. */
export const VARIANTS = Object.freeze({
 nominal_full:'Номинальная модель · 1,80 кг / μ=1 / 0,7 Н·м',
 upper_mass_full:'Увеличенная масса · 2,08 кг',
 friction_035_full:'Слабое трение · μ=0,35',
 torque_035_full:'Слабые приводы · 0,35 Н·м'
});
export const LEGS=['ml','bl','br','mr','fr','fl'];
export const LABELS={
 stand_neutral:'Стойка · 6 опор',stand_yaw_plus15:'Корпус: разворот +15°',stand_yaw_minus15:'Корпус: разворот −15°',
 stand_roll15:'Корпус: крен 15°',stand_pitch15:'Корпус: тангаж 15°',stand_roll_pitch10:'Корпус: крен + тангаж 10°',
 stand_height_plus20:'Корпус: поднять 20 мм',stand_height_minus20:'Корпус: опустить 20 мм',
 stand_shift_forward40:'Корпус: сдвиг вперёд 40 мм',stand_shift_left40:'Корпус: сдвиг влево 40 мм',
 stand_yaw_roll:'Корпус: разворот + крен',tripod_forward:'Tripod: вперёд',tripod_backward:'Tripod: назад',tripod_strafe:'Tripod: вбок',
 tripod_turn:'Tripod: поворот на месте',tripod_forward_turn:'Tripod: вперёд + поворот',tripod_forward_roll:'Tripod: ходьба + крен',
 tripod_forward_head:'Tripod: ходьба + голова',tripod_forward_height10:'Tripod: ходьба + высота',tripod_forward_fast:'Tripod: быстро',tripod_forward_slow:'Tripod: медленно',tripod_turn_slow:'Tripod: поворот медленно',
 wave_forward:'Wave: вперёд',wave_strafe:'Wave: вбок',wave_turn:'Wave: поворот на месте',wave_forward_roll:'Wave: ходьба + крен',wave_forward_head:'Wave: ходьба + голова',wave_forward_slow:'Wave: медленно',wave_turn_slow:'Wave: поворот медленно'};
export function validatePack(k){
 if(k?.format!=='hexapod-actual-mujoco-v12-replay-v13'||!(k.variant in VARIANTS)||k.hz!==50||!k.runs||Object.keys(k.runs).length<1)throw Error('Не тот набор MuJoCo V12');
 for(const [id,r] of Object.entries(k.runs)){
  if(!/^[a-z0-9_]+$/.test(id)||!Array.isArray(r.frames)||r.frames.length<25||!r.summary||r.frames[0][0]!==0)throw Error('Неполный физический прогон '+id);
  let last=-1;
  for(const a of r.frames){
   if(a.length!==36||a.some(v=>!Number.isFinite(v))||a[0]<=last||a[28]<0||a[28]>63||!Number.isInteger(a[28]))throw Error('Повреждены физические кадры '+id);
   const q=Math.hypot(...a.slice(4,8));if(Math.abs(q-1)>0.005)throw Error('Ориентация физики повреждена');
   last=a[0];
  }
  if(Math.abs(last-r.duration_s)>.021)throw Error('Неполная запись времени '+id);
 }
 return true;
}
const normalize=a=>{const n=Math.hypot(...a);return a.map(x=>x/n);};
export function slerpWxyz(a,b,t){
 let rhs=b.slice(),d=a.reduce((s,v,i)=>s+v*rhs[i],0);
 if(d<0){rhs=rhs.map(x=>-x);d=-d;}
 if(d>.9995)return normalize(a.map((x,i)=>x+t*(rhs[i]-x)));
 const omega=Math.acos(Math.min(1,d)),sn=Math.sin(omega);
 const wa=Math.sin((1-t)*omega)/sn,wb=Math.sin(t*omega)/sn;
 return a.map((x,i)=>wa*x+wb*rhs[i]);
}
export function frameAt(r,t){
 if(!r?.frames?.length)throw Error('Нет кадров');
 const F=r.frames,sec=Math.max(0,Math.min(r.duration_s,Number.isFinite(t)?t:0));
 let lo=0,hi=F.length-1;
 while(lo+1<hi){const mid=(lo+hi)>>1;if(F[mid][0]<=sec)lo=mid;else hi=mid;}
 const a=F[lo],b=F[Math.min(lo+1,F.length-1)],alpha=a===b||b[0]<=a[0]?0:Math.max(0,Math.min(1,(sec-a[0])/(b[0]-a[0])));
 const p=a.slice(1,4).map((x,i)=>x+(b[i+1]-x)*alpha);
 const q=slerpWxyz(a.slice(4,8),b.slice(4,8),alpha);
 const joints=a.slice(8,28).map((x,i)=>x+(b[i+8]-x)*alpha);
 const nearest=alpha<.5?a:b;
 return {t:sec,root:p,quat:q,joints,contactMask:nearest[28],slipMmS:nearest[29],fz:nearest.slice(30,36),run:r};
}
export const contactCount=mask=>LEGS.reduce((n,_,i)=>n+((mask>>i)&1),0);
