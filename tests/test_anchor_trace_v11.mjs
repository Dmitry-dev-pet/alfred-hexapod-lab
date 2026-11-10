import assert from 'node:assert/strict';
import fs from 'node:fs';
import {GroundLock,transformFoot} from '../docs/hexapod/freenove-grounded-v11/ground-lock.js';
const kit=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/freenove-grounded-v11/planted-body-yaw-kit.json',import.meta.url)));
const trace=JSON.parse(fs.readFileSync(new URL('./fixture_toes_v11.json',import.meta.url)));
assert.equal(trace.cad_model_sha256,kit.model_sha256);
for(const [name,rows] of Object.entries(trace.cases)){
 const ground=new GroundLock(kit);let maxSlip=0,stanceMin=6;
 for(const item of rows){
  const toe=Object.fromEntries(Object.entries(item).map(([id,arr])=>[id,{x:arr[0],y:arr[1],z:arr[2]}]));
  const q=ground.step(toe);
  maxSlip=Math.max(maxSlip,q.maxSlipMm||0);stanceMin=Math.min(stanceMin,q.stanceCount);
 }
 if(name==='stand_yaw'){
  assert.equal(stanceMin,6,'all six feet should be geometric support at rest');
  assert.ok(maxSlip<.05,'body yaw should preserve grounded toes');
  assert.ok(Math.abs(ground.pose.yaw)<.01,'no walking rotation when only body turns');
 }
 if(name==='turn'){
  assert.ok(maxSlip<8,'OEM tripod rotation root should be computed from support');
  assert.ok(stanceMin>=3,'tripod planned support feet');
  assert.ok(ground.pose.yaw>.25,'robot actually turns through stepping');
 }
 console.log(name,'frames',rows.length,'maxSupportDriftMm',maxSlip.toFixed(3),'stanceMin',stanceMin,'rootYawDeg',(ground.pose.yaw*180/Math.PI).toFixed(2));
}
