import assert from 'node:assert/strict';
import fs from 'node:fs';
import {GAITS,STRIDES,Pilot,validateKit,jointTargets,cycleRow,clampTwist} from '../docs/hexapod/live-v4-gaits/kinematic-controller.js';
const k=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v4-gaits-kit.json',import.meta.url),'utf8'));
assert.equal(validateKit(k),true);
assert.deepEqual(k.stride_mm_levels,STRIDES);
assert.deepEqual(k.gaits,Object.keys(GAITS));
assert.equal(k.samples,48);
assert(k.nominal_turn_radius_m>.1);
let checked=0;
for (const gait of Object.keys(GAITS))for(const stride of STRIDES){
  for(const m of [{vx:1,vy:0,wz:0},{vx:-1,vy:0,wz:0},{vx:0,vy:1,wz:0},{vx:0,vy:-1,wz:0},{vx:0,vy:0,wz:1},{vx:0,vy:0,wz:-1}]){
    for (const phi of [0,.0125,.1,.34,.5,.77,.98]){
      const q=jointTargets(k,gait,phi,stride,m);
      assert.equal(q.length,20);
      assert(q.every(Number.isFinite));
      assert(q.slice(0,18).every(v=>Math.abs(v)<=1.7+1e-9));
      assert.deepEqual(q.slice(18),k.q_stand.slice(18));
      checked++;
    }
  }
  assert.deepEqual(jointTargets(k,gait,0,stride,{vx:0,vy:0,wz:0}),k.q_stand);
}
const q15=jointTargets(k,'wave',.2,15,{vx:1,vy:0,wz:0});
const q35=jointTargets(k,'wave',.2,35,{vx:1,vy:0,wz:0});
assert(q15.some((v,i)=>Math.abs(v-q35[i])>.001),'stride must alter foot joints');
const qWave=jointTargets(k,'wave',.08,35,{vx:1,vy:0,wz:0});
const qRipple=jointTargets(k,'ripple',.08,35,{vx:1,vy:0,wz:0});
const qTripod=jointTargets(k,'tripod',.08,35,{vx:1,vy:0,wz:0});
assert(qWave.some((v,i)=>Math.abs(v-qRipple[i])>.001));
assert(qRipple.some((v,i)=>Math.abs(v-qTripod[i])>.001));
assert(Math.abs(cycleRow(k.joint_tables.wave['35'].fwd,-.0001)[0]-cycleRow(k.joint_tables.wave['35'].fwd,.9999)[0])<1e-6);
for(const [stride,expected] of [[15,.015],[25,.025],[35,.035]]){
  const pilot=new Pilot(k);pilot.setGait('wave');pilot.setStride(stride);pilot.setTempo(4);
  // Integrate for sufficiently long that command ramp and stride blend settle.
  pilot.command(1,0,0);
  let prev=pilot.y;
  for(let i=0;i<160;i++){const s=pilot.tick(.025);assert(s.q.every(Number.isFinite));if(i===159)assert(Math.abs((pilot.y-prev)/.025-expected*4/2.4)<.00005);prev=pilot.y;}
  assert.equal(pilot.stride,stride);
  pilot.stop();for(let i=0;i<120;i++)pilot.tick(.025);
  assert(Object.values(pilot.current).every(v=>Math.abs(v)<1e-7));
  assert(pilot.joints.every((v,j)=>Math.abs(v-k.q_stand[j])<1e-5));
}
const p=new Pilot(k);p.command(.8,-.6,.5);for(let i=0;i<20;i++)p.tick(.025);
const at=p.phase;p.setGait('ripple');p.setStride(25);let maxJump=0;
for(let i=0;i<240;i++){
  const old=p.joints.slice();const s=p.tick(.025);
  maxJump=Math.max(maxJump, ...s.q.map((v,j)=>Math.abs(v-old[j])));
  assert(s.q.slice(0,18).every(v=>Math.abs(v)<=1.7+1e-8));
}
assert.equal(p.gait,'ripple');assert.equal(p.stride,25);
assert(maxJump<=Math.min(5.5,p.tempo*1.7)*.025+1e-8,'joint transition slew');
p.setGait('tripod');for(let i=0;i<240;i++)p.tick(.025);
assert.equal(p.gait,'tripod');
p.command(-.7,.4,-.6);for(let i=0;i<120;i++)p.tick(.025);
assert(Math.abs(p.yaw)>.001);assert(p.joints.every(Number.isFinite));
p.stop();for(let i=0;i<150;i++)p.tick(.025);
assert(p.joints.every((v,i)=>Math.abs(v-k.q_stand[i])<1e-5));
let norm=clampTwist({vx:1,vy:1,wz:1});assert(Math.abs(norm.vx+norm.vy+norm.wz-1.6)<1e-10);
assert.throws(()=>p.setGait('unknown'));
assert.throws(()=>p.setStride(50));
assert.throws(()=>p.tick(.5));
assert.throws(()=>p.command(NaN,0,0));
assert.throws(()=>p.setTempo(20));
console.log(`V4 PASSED: ${checked} combinations, gait switching, 3 stride lengths, stop, joint bounds, tempo and input safety`);
