import assert from 'node:assert/strict';
import fs from 'node:fs';
import {Pilot,validateKit,jointTargets,cycleRow} from '../docs/hexapod/live-v2-offline/kinematic-controller.js';
const k=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v2-offline-kit.json',import.meta.url),'utf8'));
assert.equal(validateKit(k),true);
assert.deepEqual(jointTargets(k,0,{vx:0,vy:0,wz:0}),k.q_stand);
let a=cycleRow(k.joint_tables.fwd,.9999);assert.equal(a.length,20);
for(let mode of ['vx','vy','wz']){
  let p=new Pilot(k),cfg={vx:0,vy:0,wz:0};cfg[mode]=1;p.command(cfg.vx,cfg.vy,cfg.wz);
  for(let i=0;i<60;i++){let s=p.tick(.05);assert.equal(s.q.length,20);assert(s.q.every(Number.isFinite));}
  assert(Math.abs(p[mode==='vx'?'y':mode==='vy'?'x':'yaw'])>0.001,mode);
  p.stop();for(let i=0;i<180;i++)p.tick(.05);
  assert(Object.values(p.current).every(v=>Math.abs(v)<1e-7),'command must decay on stop');
  assert(p.joints.every((q,j)=>Math.abs(q-k.q_stand[j])<.00001),'joint pose must return to neutral');
}
// Coupled XY + yaw, joystick direction change in the middle of a cycle.
let p=new Pilot(k);p.command(.8,-.7,.6);
for(let i=0;i<45;i++)p.tick(.025);
p.command(-.5,.4,-.6);
for(let i=0;i<45;i++){let s=p.tick(.025);assert(s.q.every(Number.isFinite));assert(s.q.slice(0,18).every(v=>Math.abs(v)<=1.7));}
p.stop();for(let i=0;i<120;i++)p.tick(.025);
assert(p.joints.every((q,j)=>Math.abs(q-k.q_stand[j])<.00001));
assert.throws(()=>p.tick(.5));
assert.throws(()=>p.command(Infinity,0,0));
console.log('Offline V2 controller JS: 8 directional, stop, boundary and mixed-command checks passed');
