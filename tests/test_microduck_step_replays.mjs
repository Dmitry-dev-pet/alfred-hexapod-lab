import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {webcrypto,createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-step-v1/pose-loader.js';
const root=new URL(process.env.MICRODUCK_STEP_DIST ? 'file://'+process.env.MICRODUCK_STEP_DIST.replace(/\/$/,'')+'/' : '../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-step-v1/replays.json',root)));
const sha=index.model_package_sha256;
const buffer=c=>{const b=readFileSync(new URL(c.url.slice(1),root));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('five clips retain successful, blocked, firm-contact and flat-control trials',()=>{
 assert.equal(validateIndex(index,sha).clips.length,5);
 assert.throws(()=>validateIndex(index,'f'.repeat(64)));
 const duplicate=structuredClone(index);duplicate.clips[4]=duplicate.clips[0];assert.throws(()=>validateIndex(duplicate,sha));
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

test('the additional contact check remains separate from the nominal success',()=>{
 const nominal=index.clips.filter(c=>c.height_m>0&&c.contact_profile==='upstream_5ms');
 assert.equal(nominal.length,3);assert.equal(nominal.filter(c=>c.status==='PASS').length,1);
 const firm=index.clips.find(c=>c.contact_profile==='firm_1ms');assert.equal(firm.seed,20261111);assert.equal(firm.status,'FAIL');assert.ok(firm.max_contact_penetration_m<.0011);
 assert.equal(index.clips.filter(c=>c.status==='CONTROL').length,1);assert.ok(index.clips.every(c=>c.fell_s===null));
});
