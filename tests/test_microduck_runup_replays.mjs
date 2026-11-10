import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-runup-roll-v1/pose-loader.js';
const root=new URL('../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-runup-roll-v1/replays.json',root)));
const sha=index.model_package_sha256;
const buffer=c=>{const b=readFileSync(new URL(c.url.slice(1),root));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('twelve source-bound clips preserve controls and unsuccessful trials',()=>{
 assert.equal(validateIndex(index,sha).clips.length,12);
 assert.throws(()=>validateIndex(index,'f'.repeat(64)));
 for(const mutate of [d=>d.clips[11]=d.clips[0],d=>d.clips[0].url='/assets/../private.bin',d=>d.roll_policy_sha256='f'.repeat(64),d=>d.clips[0].duration_s=11]){const d=structuredClone(index);mutate(d);assert.throws(()=>validateIndex(d,sha));}
 assert.equal(index.clips.filter(c=>c.height_m===0&&c.full_forward_roll&&c.standing_final_1s).length,2);
 assert.equal(index.clips.filter(c=>c.height_m>0&&c.trigger_s!==null).length,8);
 assert.equal(index.clips.filter(c=>c.trigger_s===null).length,2);
 assert.ok(index.clips.every(c=>c.quality_gate&&c.max_body_step_penetration_m<.001));
 assert.ok(index.clips.filter(c=>c.height_m>0).every(c=>!c.step_goal_reached));
});
test('all 7600 frames decode with 70 unit-quaternion poses and varying durations',async()=>{
 let n=0;for(const c of index.clips){const v=await decodePoseBytes(buffer(c),c,index);n+=v.frames.length;
 assert.equal(v.frames.length,c.frames);assert.ok(Math.abs(v.frames.at(-1)[0]-c.duration_s)<1e-5);assert.equal(v.frames[0].length,498);}
 assert.equal(n,7600);
});
test('corrupt and truncated tracks fail before playback',async()=>{
 const c=index.clips[0];const b=buffer(c);new Uint8Array(b)[100]^=1;
 await assert.rejects(()=>decodePoseBytes(b,c,index),/сумма/);
 await assert.rejects(()=>decodePoseBytes(b.slice(0,-4),c,index),/размер/);
 const wrong=buffer(c);new Uint8Array(wrong)[0]=0;const forged={...c,sha256:createHash('sha256').update(new Uint8Array(wrong)).digest('hex')};
 await assert.rejects(()=>decodePoseBytes(wrong,forged,index),/формат/);
});
test('the verified selected clip is available offline from cache',async()=>{
 const originalFetch=globalThis.fetch,oldCaches=globalThis.caches;const entries=new Map();let requests=0;
 globalThis.caches={async open(){return{async match(k){return entries.get(k)?.clone();},async put(k,v){entries.set(k,v.clone());},async delete(k){entries.delete(k);}};}};
 const c=index.clips[0];globalThis.fetch=async()=>{requests++;return new Response(buffer(c));};
 try{await loadPoseClip(c,index);globalThis.fetch=async()=>{throw Error('offline');};const p=await loadPoseClip(c,index);assert.equal(p.frames.length,c.frames);assert.equal(requests,1);}
 finally{globalThis.fetch=originalFetch;globalThis.caches=oldCaches;}
});
