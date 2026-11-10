import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {forwardKinematics} from '../docs/hexapod/microduck-control-v1/kinematics.js';
import {KinematicPilot} from '../docs/hexapod/microduck-control-v1/control-core.js';
import {DualStickInput} from '../docs/hexapod/freenove-dual-v8/dual-stick-input.js';
const read=n=>JSON.parse(readFileSync(new URL(n,process.env.MICRODUCK_CONTROL_DATA ? 'file://'+process.env.MICRODUCK_CONTROL_DATA.replace(/\/$/,'')+'/' : import.meta.url)));
const rig=read('rig.json'),kit=read('motion-kit.json'),fixtures=read('fk-fixtures.json');
test('all 70 original visual instances agree with independent MuJoCo FK fixtures',()=>{
 let pos=0,rot=0;
 for(const f of fixtures){const actual=forwardKinematics(rig,f.qpos);assert.equal(actual.length,70);
  for(let i=0;i<70;i++){const a=actual[i],b=f.poses[i];pos=Math.max(pos,...a.p.map((v,j)=>Math.abs(v-b.p[j])));const sign=a.q.reduce((s,v,j)=>s+v*b.q[j],0)<0?-1:1;rot=Math.max(rot,...a.q.map((v,j)=>Math.abs(v-sign*b.q[j])));}}
 assert.ok(pos<1e-8,'position error '+pos);assert.ok(rot<1e-8,'rotation error '+rot);
});
test('joysticks change travel, heading and head while stop decelerates to rest',()=>{
 const p=new KinematicPilot(rig,kit),neutral=p.pose.slice();p.command(1,.5,.8,.5);
 for(let i=0;i<150;i++)p.tick(.02);
 assert.ok(Math.hypot(p.x,p.y)>.15);assert.ok(p.heading>.5);
 assert.ok(Math.abs(p.pose[rig.servo_qpos.head_yaw]-neutral[rig.servo_qpos.head_yaw])>.1);
 p.stop();const x=p.x,y=p.y;
 for(let i=0;i<50;i++){p.command(0,0,0,0);p.tick(.02);}
 assert.equal(p.x,x);assert.equal(p.y,y);assert.equal(p.state,'standing');
});
test('all three recorded fall/get-up actions preserve end-to-start pose continuity',()=>{
 for(const name of Object.keys(kit.actions)){
  const p=new KinematicPilot(rig,kit);p.command(1,.2);for(let i=0;i<100;i++)p.tick(.02);
  assert.equal(p.fall(name),true);assert.equal(p.rise(),false);assert.equal(p.fall(name),false);
  for(let i=0;i<70;i++)p.tick(.02);assert.equal(p.state,'down');
  const fallen=p.pose.slice();assert.equal(p.rise(),true);p.tick(0);
  assert.ok(Math.max(...p.pose.map((v,i)=>Math.abs(v-fallen[i])))<1e-7);
  for(let i=0;i<330;i++)p.tick(.02);assert.equal(p.state,'standing');
  assert.ok(p.pose[2]>.09&&p.pose[2]<.19);assert.ok(p.pose.every(Number.isFinite));
 }
});
test('stop pauses a recorded action and resume completes it; reset clears action state',()=>{
 const p=new KinematicPilot(rig,kit);p.fall('side90');for(let i=0;i<20;i++)p.tick(.02);p.stop();const hold=p.pose.slice();
 for(let i=0;i<30;i++)p.tick(.02);assert.deepEqual(p.pose,hold);assert.equal(p.paused,true);
 p.resume();for(let i=0;i<80;i++)p.tick(.02);assert.equal(p.state,'down');p.rise();for(let i=0;i<20;i++)p.tick(.02);
 p.reset();assert.equal(p.action,null);assert.equal(p.state,'standing');assert.equal(p.x,0);assert.equal(p.settle,null);
});
test('two pointers stay independent and cancel/release cannot leave a stale command',()=>{
 const s=new DualStickInput();assert.equal(s.start('left',1,.5,1),true);assert.equal(s.start('right',2,-.5,.2),true);
 assert.equal(s.end('left',1),true);assert.equal(s.right.pointer,2);assert.notEqual(s.right.x,0);
 s.end('right',2);assert.equal(s.right.x,0);assert.equal(s.right.y,0);
 s.start('left',3,1,1);s.reset();assert.equal(s.left.pointer,null);assert.equal(s.right.pointer,null);
});
test('walking cycles are visibly nontrivial and backward derivation is disclosed',()=>{
 for(const name of ['forward','turn_left','turn_right']){
  const c=kit.cycles[name],joint=rig.servo_qpos.left_knee,v=c.qpos.map(q=>q[joint]);
  assert.ok(Math.max(...v)-Math.min(...v)>.04,name+' must move legs');
 }
 assert.equal(kit.cycles.backward.derivation,'kinematic_time_reverse_of_forward');
});
