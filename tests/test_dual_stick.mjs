import assert from 'node:assert/strict';
import {DualStickInput,normalizedPad} from '../docs/hexapod/freenove-dual-v8/dual-stick-input.js';
import {FactoryPilot,validateFactoryKit} from '../docs/hexapod/freenove-original-v7/factory-controller.js';
import fs from 'node:fs';
const factory=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/freenove-original-v7-kit.json',import.meta.url),'utf8'));
const ref=JSON.parse(fs.readFileSync(new URL('../docs/hexapod/data/live-v4-gaits-kit.json',import.meta.url),'utf8'));
assert.equal(validateFactoryKit(factory,ref),true);
let checks=0;
const c=(condition,msg)=>{assert.ok(condition,msg);checks++};
const approximate=(a,b)=>Math.abs(a-b)<1e-10;
let v=normalizedPad(1,1);c(Math.abs(Math.hypot(v.x,v.y)-1)<1e-9,'diag clamp');
c(normalizedPad(.05,.01).x===0,'deadzone');
c(normalizedPad(NaN,1).y===0,'nan guard');
const sticks=new DualStickInput();
c(sticks.start('left',11,.5,1),'left starts');
c(sticks.start('right',12,-.75,.8),'right starts');
c(!sticks.start('left',99,1,1),'one pointer per pad');
c(!sticks.start('right',11,0,1),'no duplicate pointer');
let q=sticks.sample('walk');
c(q.forward>0&&q.lateral<0,'left forward and strafe');
c(q.turn>0,'right x -> CCW');
c(approximate(q.turn,-sticks.right.x),'right walk Y ignored');
c(!sticks.move('left',99,-1,-1),'wrong ID move ignored');
c(sticks.move('right',12,1,.4),'right moves');
c(sticks.sample('walk').turn<0,'right motion changes yaw while left stays');
c(sticks.end('right',12),'right release');
c(sticks.sample('walk').turn===0&&sticks.sample('walk').forward>0,'left survives right release');
c(sticks.start('right',13,1,.5),'right restarts');
c(sticks.end('left',11),'left release');
c(sticks.sample('walk').forward===0&&sticks.sample('walk').turn<0,'right survives left release');
c(!sticks.end('left',11),'double release harmless');
q=sticks.sample('body');c(q.roll>0&&q.pitch>0,'body two axis tilt');
c(sticks.start('left',17,-1,.5),'body translate');
q=sticks.sample('body');c(q.forward>0&&q.lateral>0&&q.roll>0,'simultaneous independent body axes');
sticks.reset();q=sticks.sample('walk');c(q.forward===0&&q.lateral===0&&q.turn===0,'zero after reset');
q=sticks.sample('walk',{forward:1,left:0,turn:-1});c(q.forward===1&&q.turn===-1,'keyboard fallback');
c(sticks.start('right',21,0,1),'walk vertical-only right');
c(sticks.sample('walk').turn===0,'vertical right walk neutral');
sticks.reset();
// Integrate the independent commands into the immutable factory controller.
const pilot=new FactoryPilot(factory,ref);
sticks.start('left',1,0,1);sticks.start('right',2,.65,0);
for(let i=0;i<32;i++){const cmd=sticks.sample('walk');pilot.command(cmd.forward,cmd.lateral,cmd.turn);pilot.tick(.016);}
c(pilot.y>0&&pilot.yaw<0,'forward + right yaw together');
sticks.end('right',2);
for(let i=0;i<40;i++){const cmd=sticks.sample('walk');pilot.command(cmd.forward,cmd.lateral,cmd.turn);pilot.tick(.016);}
c(pilot.y>0,'still moving forward without right thumb');
c(pilot.now.turn===0,'yaw command decayed to zero');
sticks.end('left',1);pilot.stop();for(let i=0;i<100;i++)pilot.tick(.016);
c(pilot.now.forward===0&&pilot.now.turn===0,'both stopped smoothly');
pilot.setMode('body');sticks.start('left',5,-.5,.5);sticks.start('right',6,1,-.5);
for(let i=0;i<45;i++){
 const cmd=sticks.sample('body');pilot.setBody('forward',cmd.forward);pilot.setBody('left',cmd.lateral);pilot.setBody('roll',cmd.roll);pilot.setBody('pitch',cmd.pitch);pilot.tick(.016);
}
c(pilot.body.roll>0&&pilot.body.pitch<0&&pilot.body.forward>0,'combined pose update');
sticks.end('right',6);for(let i=0;i<60;i++){
 const cmd=sticks.sample('body');pilot.setBody('forward',cmd.forward);pilot.setBody('left',cmd.lateral);pilot.setBody('roll',cmd.roll);pilot.setBody('pitch',cmd.pitch);pilot.tick(.016);
}
c(pilot.body.roll===0&&pilot.body.pitch===0&&pilot.body.forward>0,'body right release leaves translation');
console.log('V8 dual-stick tests PASSED:',checks);
