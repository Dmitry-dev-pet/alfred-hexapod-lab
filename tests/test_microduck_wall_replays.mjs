import {cameraOutsideTerrain} from '../docs/hexapod/microduck-wall-v1/safe-camera.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {webcrypto,createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-wall-v1/pose-loader.js';
const root=new URL('../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-wall-v1/replays.json',root)));
const sha=index.model_package_sha256;
const buffer=c=>{const b=readFileSync(new URL(c.url.slice(1),root));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('nine corrected clips include a flat recovery control',()=>{
 assert.equal(validateIndex(index,sha).clips.length,9);
 assert.throws(()=>validateIndex(index,'f'.repeat(64)));
 const duplicate=structuredClone(index);duplicate.clips[8]=duplicate.clips[0];assert.throws(()=>validateIndex(duplicate,sha));
 const unsafe=structuredClone(index);unsafe.clips[0].url='/assets/../private.bin';assert.throws(()=>validateIndex(unsafe,sha));
});
test('every complete physical track decodes with 70 unit-quaternion part poses',async()=>{
 for(const c of index.clips){const v=await decodePoseBytes(buffer(c),c,index);
  assert.equal(v.frames.length,c.frames);assert.equal(Math.abs(v.frames.at(-1)[0]-c.duration_s)<1e-5,true);assert.equal(v.frames[0].length,498);
 }
});
test('tampered, truncated and wrong-header tracks fail before playback',async()=>{
 const c=index.clips[0];const b=buffer(c);new Uint8Array(b)[100]^=1;
 await assert.rejects(()=>decodePoseBytes(b,c,index),/сумма/);
 await assert.rejects(()=>decodePoseBytes(b.slice(0,-4),c,index),/размер/);
 const wrong=buffer(c);new Uint8Array(wrong)[0]=0;const forged={...c,sha256:createHash('sha256').update(new Uint8Array(wrong)).digest('hex')};
 await assert.rejects(()=>decodePoseBytes(wrong,forged,index),/формат/);
});
test('a viewed clip is reused offline from the verified browser cache',async()=>{
 const originalFetch=globalThis.fetch,oldCaches=globalThis.caches;const entries=new Map();let requests=0;
 globalThis.caches={async open(){return{async match(k){return entries.get(k)?.clone();},async put(k,v){entries.set(k,v.clone());},async delete(k){entries.delete(k);}};}};
 const c=index.clips[0];globalThis.fetch=async()=>{requests++;return new Response(buffer(c));};
 try{await loadPoseClip(c,index);globalThis.fetch=async()=>{throw Error('offline');};const p=await loadPoseClip(c,index);
  assert.equal(p.frames.length,c.frames);assert.equal(requests,1);
 }finally{globalThis.fetch=originalFetch;globalThis.caches=oldCaches;}
});

test('complete environment contacts withdraw the previous edge-stance claim',()=>{
 assert.equal(index.clips.length,9);assert.equal(index.environment_mesh_count,60);
 assert.ok(index.clips.every(c=>c.contact_profile==='complete_env_05ms'&&!c.step_goal_reached&&!c.supported_top_stance));
 assert.ok(index.clips.every(c=>c.max_body_step_penetration_m<.0003));
 assert.equal(index.clips.find(c=>c.id==='flat-h0').status,'PASS');
 const case5=index.clips.find(c=>c.id==='close-h50');assert.ok(!case5.standing_final_1s&&case5.max_body_step_penetration_m<.0003);
 const legacy=structuredClone(index);legacy.schema='microduck-back-to-wall-replays/v1';assert.throws(()=>validateIndex(legacy,sha));
});

test('the orbit camera cannot enter the 5 cm step or the floor',()=>{
 const terrain={height_m:.05,center_xy_m:[.6294373124348494,0],half_size_xy_m:[.6,.6]};
 const clipped=cameraOutsideTerrain([.05,.03,.015],terrain);assert.ok(clipped[2]>.05+.006);assert.equal(clipped[0],.05);
 const outside=[-.3,-.4,.3];assert.deepEqual(cameraOutsideTerrain(outside,terrain),outside);
 assert.ok(cameraOutsideTerrain([-.3,0,-.1],terrain)[2]>.006);
 const tall={...terrain,height_m:.25};assert.ok(cameraOutsideTerrain([.05,0,.13],tall)[2]>.25+.006);
});
