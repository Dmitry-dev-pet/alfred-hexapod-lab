import test from 'node:test';
import {gzipSync,gunzipSync} from 'node:zlib';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-step-goal-v1/pose-loader.js';
const root=new URL('../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-step-goal-v1/replays.json',root)));
const sha=index.model_package_sha256;
const packageBytes=c=>readFileSync(new URL(c.url.slice(1),root));
const buffer=c=>{const doc=JSON.parse(packageBytes(c));const b=Buffer.from(doc.tracks[c.track_sha256].data,'base64');return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('37 recordings retain goal versus quality outcomes and all tested heights',()=>{
 assert.equal(validateIndex(index,sha).clips.length,37);
 assert.deepEqual([...new Set(index.clips.map(c=>c.height_m))].sort((a,b)=>a-b),[0,.01,.02,.03,.05]);
 assert.equal(index.clips.filter(c=>c.height_m>0&&c.status==='PASS').length,7);
 assert.equal(index.clips.filter(c=>c.step_goal_reached&&!c.quality_gate).length,17);
 assert.equal(index.clips.find(c=>c.id===index.default_clip).height_m,.01);
 for(const mutate of [d=>d.clips[36]=d.clips[0],d=>d.clips[0].url='/assets/../private.bin',d=>d.clips[0].status='FAIL',d=>d.default_clip='height-2cm',d=>d.clips[1].terrain.height_m=.05]){
  const d=structuredClone(index);mutate(d);assert.throws(()=>validateIndex(d,sha));
 }
});
test('all 16752 source-bound frames decode as 70 original mesh poses',async()=>{
 let total=0;
 for(const c of index.clips){const v=await decodePoseBytes(buffer(c),c,index);total+=v.frames.length;
  assert.equal(v.frames.length,c.frames);assert.equal(v.frames[0].length,498);assert.ok(Math.abs(v.frames.at(-1)[0]-c.duration_s)<1e-5);}
 assert.equal(total,16752);
});
test('hash corruption, truncated data and forged headers are rejected',async()=>{
 const c=index.clips[0];let b=buffer(c);new Uint8Array(b)[100]^=1;
 await assert.rejects(()=>decodePoseBytes(b,c,index),/сумма/);
 await assert.rejects(()=>decodePoseBytes(b.slice(0,-4),c,index),/размер/);
 const raw=gunzipSync(Buffer.from(buffer(c)));raw[0]=0;const packed=gzipSync(raw);b=packed.buffer.slice(packed.byteOffset,packed.byteOffset+packed.byteLength);
 const forged={...c,size:b.byteLength,sha256:createHash('sha256').update(packed).digest('hex'),track_sha256:createHash('sha256').update(raw).digest('hex')};
 await assert.rejects(()=>decodePoseBytes(b,forged,index),/кадров|формат/);
});
test('selected verified recording reopens offline from the viewed-clip cache',async()=>{
 const oldFetch=globalThis.fetch,oldCaches=globalThis.caches;const entries=new Map();let requests=0;
 globalThis.caches={async open(){return{async match(k){return entries.get(k)?.clone();},async put(k,v){entries.set(k,v.clone());},async delete(k){entries.delete(k);}};}};
 const c=index.clips.find(c=>c.id===index.default_clip);
 globalThis.fetch=async()=>{requests++;return new Response(packageBytes(c));};
 try{await loadPoseClip(c,index);globalThis.fetch=async()=>{throw Error('offline');};const v=await loadPoseClip(c,index);assert.equal(v.frames.length,c.frames);assert.equal(requests,1);}
 finally{globalThis.fetch=oldFetch;globalThis.caches=oldCaches;}
});
test('the unchanged live GitHub worker serves the complete new JSON-backed collection',async()=>{
 const source=readFileSync(new URL('fixtures/github-live-sw-v3.js',import.meta.url),'utf8');
 const catalog=JSON.parse(readFileSync(new URL('hexapod/library.json',root)));
 const entry=catalog.entries.find(e=>e.id==='microduck-step-goal-v1');
 assert.equal(entry.files.length,20);
 const sha='0123456789abcdef0123456789abcdef01234567',handlers={},cache=new Map(),requests=[];
 const upstream=async input=>{
  const url=String(input);requests.push(url);
  if(url.includes('/branches/'))return new Response(JSON.stringify({commit:{sha}}));
  if(url.endsWith('/docs/hexapod/library.json'))return new Response(JSON.stringify(catalog));
  const prefix='https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/'+sha+'/docs/';
  if(url.startsWith(prefix))return new Response(readFileSync(new URL(url.slice(prefix.length),root)));
  throw Error('Unexpected bundled Site fallback: '+url);
 };
 const ctx=vm.createContext({self:{location:{origin:'https://robot.example',href:'https://robot.example/github-live-sw.js'},
  skipWaiting:async()=>{},clients:{claim:async()=>{}},addEventListener:(name,fn)=>handlers[name]=fn},
  caches:{open:async()=>({match:async k=>cache.get(String(k))?.clone(),put:async(k,v)=>cache.set(String(k),v.clone())})},
  fetch:upstream,TextDecoder,TextEncoder,Response,Request,crypto:webcrypto,URL,Date,Map,Set,Number,Uint8Array,ArrayBuffer,Promise,console});
 vm.runInContext(source,ctx);
 const request=async path=>{let response;handlers.fetch({request:new Request('https://robot.example'+path),respondWith:r=>response=Promise.resolve(r)});assert.ok(response);return response;};
 assert.equal((await request('/hexapod/library.json')).headers.get('X-Alfred-Catalog-Source'),'github');
 for(const file of entry.files){const response=await request('/'+file.path);assert.equal(response.status,200);assert.ok(await response.text());}
 assert.ok(requests.filter(url=>url.includes('/docs/hexapod/microduck-step-goal-v1/')).length>=20);
 assert.ok(requests.every(url=>!url.includes('/assets/robot-replays/')));
});
