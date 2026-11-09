import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {webcrypto,createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-falls-v1/pose-loader.js';
const root=new URL('../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-falls-v1/replays.json',root)));
const sha=index.model_package_sha256;
const buffer=c=>{const b=readFileSync(new URL(c.url.slice(1),root));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('twenty-four clips retain every candidate and step outcome',()=>{
 assert.equal(validateIndex(index,sha).clips.length,24);
 assert.throws(()=>validateIndex(index,'f'.repeat(64)));
 const duplicate=structuredClone(index);duplicate.clips[12]=duplicate.clips[0];assert.throws(()=>validateIndex(duplicate,sha));
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

test('directed falling and platform success are kept separate',()=>{
 assert.equal(index.clips.length,24);
 assert.equal(index.clips.filter(c=>!c.height_m&&c.status==='PASS').length,6);
 assert.equal(index.clips.filter(c=>c.height_m&&c.status==='PASS').length,1);
 const winner=index.clips.find(c=>c.id==='step10-left');assert.ok(winner.step_goal_reached&&winner.directed_fall&&winner.standing_final_1s);
 for(const direction of ['back','left','right'])assert.ok(index.clips.some(c=>c.direction===direction&&!c.height_m&&c.status==='PASS'));
 assert.ok(index.clips.filter(c=>c.height_m===.02).every(c=>!c.step_goal_reached));
 assert.ok(index.clips.every(c=>c.first_fall_in_requested_cone||!c.directed_fall));
});
