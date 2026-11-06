import assert from 'node:assert/strict';
import fs from 'node:fs';
import {FactoryPilot,GAITS,validateFactoryKit,oemWalkingTargets,oemBodyTargets} from '../docs/hexapod/freenove-original-v7/factory-controller.js';
const k=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/freenove-original-v7-kit.json',import.meta.url),'utf8'));
const ref=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v4-gaits-kit.json',import.meta.url),'utf8'));
assert.equal(validateFactoryKit(k,ref),true);
assert.equal(k.factory_commands.move_xy_clamp_mm,35);
assert.equal(k.factory_commands.lift_mm,40);
assert.equal(k.factory_commands.position_xy_clamp_mm,40);
assert.equal(k.factory_commands.position_z_clamp_mm,20);
assert.equal(k.factory_commands.attitude_clamp_deg,15);
let checks=0;
const close=(a,b,epsilon=1e-5)=>Math.max(...a.map((x,i)=>Math.abs(x-b[i])))<epsilon;
for(const gait of Object.keys(GAITS)){
 const g=k.gaits[gait];assert(g.cycle_s>.3&&g.cycle_s<1.2);
 for(const direction of ['forward','back','left','right','forward_left','forward_right','back_left','back_right','ccw','cw','lift']){
  assert(g.rows[direction].length===48);
  for(const q of g.rows[direction]){assert(q.length===20);assert(q.every(Number.isFinite));assert(q.slice(0,18).every(x=>Math.abs(x)<=k.cad_adapter.joint_ik_bound_rad+1e-6));}
  checks++;
 }
 const moving=oemWalkingTargets(k,gait,.45,.8,.4,.25);assert(moving.some((x,i)=>Math.abs(x-k.q_home[i])>.002));checks++;
 const stop=oemWalkingTargets(k,gait,.2,0,0,0);assert.deepEqual(stop,k.q_home);checks++;
 for(const axis of ['forward','left','turn']){
  const p=new FactoryPilot(k,ref);p.setGait(gait);p.command(axis==='forward'?1:0,axis==='left'?1:0,axis==='turn'?1:0);
  for(let i=0;i<60;i++)p.tick(.015);
  assert(Math.hypot(p.x,p.y, p.yaw)>.002);
  p.stop();for(let i=0;i<100;i++)p.tick(.02);
  assert(close(p.displayQ,k.q_home,.0002),'should return to factory home');
  checks++;
 }
}
for(const axis of ['forward','left','height','roll','pitch','yaw']){
 for(const sign of [-1,1]){
  const p=new FactoryPilot(k,ref);p.setMode('body');p.setBody(axis,sign);
  for(let i=0;i<90;i++)p.tick(.02);
  assert(close(p.displayQ,k.body_axes[axis==='roll'||axis==='pitch'||axis==='yaw'?axis+'_rad':axis+'_m'][sign>0?'pos':'neg'],.0001));
  assert(p.displayQ.slice(0,18).every(x=>Math.abs(x)<=2.4+1e-5));
  p.stop();for(let i=0;i<90;i++)p.tick(.02);
  assert(close(p.displayQ,k.q_home,.0001));checks++;
 }
}
{
 const p=new FactoryPilot(k,ref);p.command(.8,.7,.5);
 for(let i=0;i<40;i++)p.tick(.02);
 p.command(-.9,-.4,-.5);
 for(let i=0;i<40;i++){const s=p.tick(.02);assert(s.q.every(Number.isFinite));assert(s.q.slice(0,18).every(q=>Math.abs(q)<=2.4+1e-8));}
 p.setGait('wave');p.tick(.02);p.setMode('body');p.setBody('height',1);p.setBody('pitch',1);
 for(let i=0;i<100;i++)p.tick(.02);
 assert(Math.abs(p.tick(.02).rootOffsetWorld[2]-.02)<.0001);
 p.reset();assert.deepEqual(p.displayQ,k.q_home);checks++;
}
assert.throws(()=>new FactoryPilot({...k,format:'WRONG'},ref));checks++;
assert.throws(()=>new FactoryPilot(k,ref).tick(1));checks++;
assert.throws(()=>new FactoryPilot(k,ref).setBody('unknown',1));checks++;
console.log('Freenove Original V7 JS checks PASSED',checks);
