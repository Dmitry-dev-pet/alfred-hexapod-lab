import assert from 'node:assert/strict';
import fs from 'node:fs';
import {MotionPilot,validateMotionKit,normalizeAxes,poseJointTargets} from '../docs/hexapod/live-v5-motions/motion-controller.js';
const base=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v4-gaits-kit.json',import.meta.url),'utf8'));
const kit=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v5-motion-kit.json',import.meta.url),'utf8'));
assert.equal(validateMotionKit(kit,base),true);
const stand=base.q_stand;
const maxdist=(a,b)=>Math.max(...a.map((v,i)=>Math.abs(v-b[i])));
let checks=0;
function settle(p,t=1.2){let state;for(let i=0;i<t/.02;i++)state=p.tick(.02);return state;}
for(const axis of ['forward','left','height','roll','pitch','yaw']){
  for(const sign of [-1,1]){
    const p=new MotionPilot(base,kit);p.setMode('body');p.setBody(axis,sign);
    const s=settle(p);
    assert(s.mode==='body');assert(Math.abs(s.body[axis])>.99);
    assert(maxdist(s.q,stand)>.001,axis);
    assert(s.q.slice(0,18).every(v=>Math.abs(v)<=1.7));
    if(axis==='height')assert(Math.abs(s.rootOffsetWorld[2])>.011);
    if(['roll','pitch','yaw'].includes(axis))assert(Math.hypot(...s.bodyRotvecWorld)>.07);
    p.stopAll();const end=settle(p,2);assert(maxdist(end.q,stand)<.00001,'body stop returns stand');
    checks++;
  }
}
for(const code of base.leg_order){
  for(const axis of ['forward','left','lift']){
    const p=new MotionPilot(base,kit);p.setMode('leg');p.setLeg(code);p.setFoot(axis,1);
    const s=settle(p),idx=kit.leg_joint_indices[code];
    assert(idx.some(i=>Math.abs(s.q[i]-stand[i])>.001));
    assert(s.q.slice(0,18).every(v=>Math.abs(v)<=1.7));
    const other=Array.from({length:18},(_,i)=>i).filter(i=>!idx.includes(i));
    assert(other.every(i=>Math.abs(s.q[i]-stand[i])<.00001),'Only selected leg changes');
    p.stopAll();assert(maxdist(settle(p,2).q,stand)<.00001);
    checks++;
  }
}
for(const axis of ['yaw','pitch']){
  const p=new MotionPilot(base,kit);p.setMode('head');p.setHead(axis,1);
  const s=settle(p);assert(Math.abs(s.q[kit.head_joint_indices[axis]]-stand[kit.head_joint_indices[axis]])>.1);
  assert(s.q.slice(0,18).every((v,i)=>Math.abs(v-stand[i])<1e-6));
  p.stopAll();assert(maxdist(settle(p,2).q,stand)<.00001);
  checks++;
}
for(const gait of ['wave','ripple','tripod']){
 const p=new MotionPilot(base,kit);p.setGait(gait);p.setStride(35);p.setTempo(4);
 p.control(1,.4,.25);let s=settle(p,3);
 assert(s.mode==='walk');assert(Math.hypot(s.x,s.y)>.02);
 assert(s.q.slice(0,18).every(v=>Number.isFinite(v)&&Math.abs(v)<=1.7));
 p.stopAll();s=settle(p,3);
 assert(s.mode==='walk');assert(!s.moving);
 checks++;
}
for(const mode of ['walk','body','leg','head']){
 const p=new MotionPilot(base,kit);p.setMode(mode);p.control(.8,-.7,.3);
 settle(p,.4);p.setMode(mode==='walk'?'body':'walk');
 let s=settle(p,2);assert(s.q.every(Number.isFinite));assert(s.q.slice(0,18).every(v=>Math.abs(v)<=1.7));
 checks++;
}
const p=new MotionPilot(base,kit);p.setMode('body');p.setBody('height',3);assert(p.tick(.02).body.height<=1);
assert.equal(Object.values(normalizeAxes({forward:1,left:1,yaw:1},1.3)).reduce((a,b)=>a+Math.abs(b),0).toFixed(2),'1.30');
assert.throws(()=>p.tick(.5));assert.throws(()=>p.setBody('strange',1));assert.throws(()=>p.setLeg('unknown'));
assert(maxdist(poseJointTargets(base,kit,{forward:0,left:0,height:0,roll:0,pitch:0,yaw:0},null,{}),stand)<1e-8);
console.log('CAD30 OFFLINE V5: motion mode checks passed',checks,'+invalid-input guards');
