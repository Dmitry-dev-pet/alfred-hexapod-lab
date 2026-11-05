import assert from 'node:assert/strict';
import fs from 'node:fs';
import {Pilot,validateKit,jointTargets} from '../docs/hexapod/live-v3-fast/kinematic-controller.js';
const kit=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v2-offline-kit.json', import.meta.url),'utf8'));
assert.equal(validateKit(kit),true);
const run=(tempo,key,steps=240)=>{
 const p=new Pilot(kit);p.setTempo(tempo);
 const cmd={vx:0,vy:0,wz:0};cmd[key]=1;
 p.command(cmd.vx,cmd.vy,cmd.wz);
 for(let i=0;i<steps;i++){
  const state=p.tick(.025);
  assert(state.q.every(Number.isFinite));
  assert(state.q.slice(0,18).every(x=>Math.abs(x)<=1.700001));
  assert.equal(state.tempo,tempo);
 }
 return p;
};
for(const axis of ['vx','vy','wz']){
 const path=axis==='vx'?'y':axis==='vy'?'x':'yaw';
 const slow=run(1,axis),fast=run(4,axis),max=run(8,axis);
 assert(Math.abs(slow[path])>0.001);
 assert(Math.abs(fast[path]/slow[path]-4)<1e-9,[axis,'4x',fast[path]/slow[path]]);
 assert(Math.abs(max[path]/slow[path]-8)<1e-9,[axis,'8x',max[path]/slow[path]]);
}
const p=new Pilot(kit);p.setTempo(8);p.command(.9,-.7,.6);
for(let i=0;i<60;i++)p.tick(.03);
p.command(-.8,.2,-.4);
for(let i=0;i<60;i++)p.tick(.03);
assert(p.joints.every(Number.isFinite));
p.stop();for(let i=0;i<150;i++)p.tick(.03);
assert(p.joints.every((v,i)=>Math.abs(v-kit.q_stand[i])<1e-5));
assert(Object.values(p.current).every(v=>Math.abs(v)<1e-7));
assert.throws(()=>p.setTempo(9));
assert.throws(()=>p.setTempo(0));
assert.throws(()=>p.setTempo(NaN));
assert.deepEqual(jointTargets(kit,0,{vx:0,vy:0,wz:0}),kit.q_stand);
console.log('Fast offline joystick: x1/x4/x8 proportional XY/Yaw, slew/stop/change-direction and bounds PASS');
