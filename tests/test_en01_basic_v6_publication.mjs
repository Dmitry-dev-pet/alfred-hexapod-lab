import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {webcrypto} from 'node:crypto';
import {pathToFileURL} from 'node:url';
import {test} from 'node:test';
globalThis.crypto??=webcrypto;
const root=resolve(process.argv[2]),site=resolve(process.argv[3]);
const {validateIndex,decodeRecord,loadVerified}=await import(pathToFileURL(root+'/docs/hexapod/en01-basic-v6/record-loader.js'));
const index=JSON.parse(await readFile(root+'/docs/hexapod/en01-basic-v6/replays.json'));
const bytes=async clip=>{const b=await readFile(site+clip.url);return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('All twelve baseline records decode and all nine failures remain visible',async()=>{
  validateIndex(index);let frames=0;
  for(const clip of index.clips){const d=await decodeRecord(await bytes(clip),clip,index);frames+=d.frames.length;assert.equal(d.basic_motion.cad_surface_audit.unexpected_intersection_frames,0);assert.equal(d.result.basic_pass,clip.basic_pass);}
  assert.equal(frames,3600);assert.equal(Object.keys(index.rig.part_body_map).length,73);
});
test('Wrong bytes, truncation, and source identity are rejected',async()=>{
  const c=index.clips[0],b=await bytes(c);const corrupted=b.slice(0);new Uint8Array(corrupted)[30]^=1;
  await assert.rejects(()=>decodeRecord(corrupted,c,index),/сумма/);await assert.rejects(()=>decodeRecord(b.slice(1),c,index),/размер/);
  await assert.rejects(()=>decodeRecord(b,{...c,trace_sha256:'0'.repeat(64)},index),/исходному/);
});
test('Duplicate, missing and unsafe catalog entries are rejected',()=>{
  for(const edit of [d=>d.clips.pop(),d=>d.clips[1]=d.clips[0],d=>d.clips[0].url='/assets/robot-replays/en01-basic-v6/../secret.json']){const d=structuredClone(index);edit(d);assert.throws(()=>validateIndex(d));}
});
test('A viewed recording reopens without a network connection',async()=>{
  const c=index.clips[0],b=await bytes(c),memory=new Map();let requests=0;
  globalThis.caches={open:async()=>({match:async k=>memory.get(k)?.clone(),put:async(k,v)=>memory.set(k,v.clone()),delete:async k=>memory.delete(k)})};
  globalThis.fetch=async()=>{requests++;return new Response(b);};
  const read=()=>loadVerified(c.url,c.size,c.sha256,a=>decodeRecord(a,c,index));
  await read();globalThis.fetch=async()=>{throw Error('offline');};const d=await read();assert.equal(d.frame_count,300);assert.equal(requests,1);
});
