/** Exact V11 *joint command* trace for offline MuJoCo. Never prescribes free base or contacts.
 * Copyright attribution: factory gait tables ported from Freenove FNK0052 CC BY-NC-SA 3.0.
 */
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {PlantedCombinedPilot} from '../../docs/hexapod/freenove-grounded-v11/planted-controller.js';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const read=(name)=>JSON.parse(fs.readFileSync(path.join(root,name),'utf8'));
const factory=read('docs/hexapod/data/freenove-original-v7-kit.json');
const ref=read('docs/hexapod/data/live-v4-gaits-kit.json');
const head=read('docs/hexapod/data/live-v5-motion-kit.json');
const planted=read('docs/hexapod/freenove-grounded-v11/planted-body-yaw-kit.json');
const dt=0.01;
const specs=[
 {id:'stand_neutral',gait:'tripod',duration_s:1.0,active_s:0,loc:{},body:{}},
 {id:'stand_yaw_plus15',gait:'tripod',duration_s:3.4,active_s:2.1,loc:{},body:{yaw:1}},
 {id:'stand_yaw_minus15',gait:'tripod',duration_s:3.4,active_s:2.1,loc:{},body:{yaw:-1}},
 {id:'stand_roll15',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{roll:1}},
 {id:'stand_roll_pitch10',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{roll:2/3,pitch:-2/3}},
 {id:'tripod_forward',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:1},body:{}},
 {id:'tripod_turn',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{turn:1},body:{}},
 {id:'tripod_forward_turn',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:0.7,turn:0.55},body:{}},
 {id:'tripod_forward_roll',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:0.7},body:{roll:0.7}},
 {id:'wave_forward',gait:'wave',duration_s:4.25,active_s:3.25,loc:{forward:1},body:{}},
 {id:'wave_turn',gait:'wave',duration_s:4.25,active_s:3.25,loc:{turn:1},body:{}},
 {id:'wave_forward_roll',gait:'wave',duration_s:4.25,active_s:3.25,loc:{forward:0.7},body:{roll:0.7}},
 {id:'stand_pitch15',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{pitch:1}},
 {id:'stand_height_plus20',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{height:1}},
 {id:'stand_height_minus20',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{height:-1}},
 {id:'stand_shift_forward40',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{forward:1}},
 {id:'stand_shift_left40',gait:'tripod',duration_s:3.2,active_s:2.0,loc:{},body:{left:1}},
 {id:'stand_yaw_roll',gait:'tripod',duration_s:3.4,active_s:2.1,loc:{},body:{yaw:1,roll:.5}},
 {id:'tripod_backward',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:-1},body:{}},
 {id:'tripod_strafe',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{left:1},body:{}},
 {id:'wave_strafe',gait:'wave',duration_s:4.25,active_s:3.25,loc:{left:1},body:{}},
 {id:'tripod_forward_fast',gait:'tripod',duration_s:4.0,active_s:3.0,tempo:2,loc:{forward:1},body:{}},
 {id:'tripod_forward_slow',gait:'tripod',duration_s:5.5,active_s:4.5,tempo:.5,loc:{forward:1},body:{}},
 {id:'tripod_turn_slow',gait:'tripod',duration_s:5.5,active_s:4.5,tempo:.5,loc:{turn:1},body:{}},
 {id:'wave_forward_slow',gait:'wave',duration_s:5.5,active_s:4.5,tempo:.5,loc:{forward:1},body:{}},
 {id:'wave_turn_slow',gait:'wave',duration_s:5.5,active_s:4.5,tempo:.5,loc:{turn:1},body:{}},
 {id:'tripod_forward_head',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:.65},body:{},head:{yaw:1,pitch:-.7}},
 {id:'wave_forward_head',gait:'wave',duration_s:4.25,active_s:3.25,loc:{forward:.65},body:{},head:{yaw:1,pitch:-.7}},
 {id:'tripod_forward_height10',gait:'tripod',duration_s:4.0,active_s:3.0,loc:{forward:.65},body:{height:.5}},
];
const output=process.argv[2]||path.join(root,'research/physics_v12/data/targets_v11.json');
const result={schema:'fnk0052-v11-exact-controller-targets-physics-v12',dt_s:dt,source:'PlantedCombinedPilot V11 actual JS; q only, NO root pose/contact constraints',cases:{}};
for(const spec of specs){
 const p=new PlantedCombinedPilot(factory,ref,head,planted);
 p.setGait(spec.gait);p.setTempo(spec.tempo||1);
 const qrows=[p.tick(0).q.map(x=>+x.toFixed(8))];
 let largest=0;
 for(let j=0;j<Math.round(spec.duration_s/dt);j++){
  const active=(j*dt)<spec.active_s;
  const loc=active?spec.loc:{};
  p.drive(loc.forward||0,loc.left||0,loc.turn||0);
  const body=active?spec.body:{};
  for(const axis of ['yaw','roll','pitch','forward','left','height'])p.setBody(axis,body[axis]||0);
  const head=active?spec.head||{}:{};for(const axis of ['yaw','pitch'])p.setHead(axis,head[axis]||0);
  const s=p.tick(dt);
  qrows.push(s.q.map(x=>+x.toFixed(8)));
  largest=Math.max(largest,...s.q.map(Math.abs));
 }
 result.cases[spec.id]={spec,targets_q20:qrows,max_abs_q_rad:largest,
   visual_dead_reckoning_end:{x_m:p.walk.x,y_m:p.walk.y,yaw_deg:p.walk.yaw*180/Math.PI}};
 console.log(spec.id,'samples',qrows.length,'maxabs',largest.toFixed(3));
}
fs.mkdirSync(path.dirname(output),{recursive:true});fs.writeFileSync(output,JSON.stringify(result)+'\n');
console.log('OUTPUT',output,fs.statSync(output).size);
