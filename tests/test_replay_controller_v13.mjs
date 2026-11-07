import assert from 'node:assert/strict';
import fs from 'node:fs';
import {validatePack,frameAt,contactCount,slerpWxyz} from '../docs/hexapod/physics-replay-v13/replay-controller.js';
const root=new URL('../docs/hexapod/data/',import.meta.url);
let num=0;
for(const variant of ['nominal_full','upper_mass_full','friction_035_full','torque_035_full']){
 const data=JSON.parse(fs.readFileSync(new URL('physics-v12-'+variant+'.json',root)));
 assert.equal(validatePack(data),true);
 for(const [name,run] of Object.entries(data.runs)){
  num++;let a=frameAt(run,0),b=frameAt(run,run.duration_s),mid=frameAt(run,run.duration_s/2);
  assert.equal(a.t,0);assert.ok(b.t>=a.t);assert.equal(a.joints.length,20);assert.equal(mid.root.length,3);
  assert.equal(mid.quat.length,4);assert.ok(Math.abs(Math.hypot(...mid.quat)-1)<1e-6);
  assert.equal(mid.fz.length,6);assert.ok(contactCount(mid.contactMask)>=0&&contactCount(mid.contactMask)<=6);
  assert.ok(mid.root.every(Number.isFinite));assert.ok(mid.joints.every(Number.isFinite));
 }
}
assert.equal(num,53);assert.deepEqual(slerpWxyz([1,0,0,0],[1,0,0,0],.5),[1,0,0,0]);
console.log('V13 actual measured 53-run replay controller PASS',num);
