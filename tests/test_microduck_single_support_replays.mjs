import test from 'node:test';
import {gzipSync,gunzipSync} from 'node:zlib';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {validateIndex,decodePoseBytes,loadPoseClip} from '../docs/hexapod/microduck-single-support-v1/pose-loader.js';
const root=new URL('../docs/',import.meta.url);
const index=JSON.parse(readFileSync(new URL('hexapod/microduck-single-support-v1/replays.json',root)));
const sha=index.model_package_sha256;
const packageBytes=c=>readFileSync(new URL(c.url.slice(1),root));
const buffer=c=>{const doc=JSON.parse(packageBytes(c));const b=Buffer.from(doc.tracks[c.track_sha256].data,'base64');return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
test('14 records preserve walking controls and all ten failed holds',()=>{
 assert.equal(validateIndex(index,sha).clips.length,14);
 assert.equal(index.clips.filter(c=>c.status==='FAIL').length,10);
 assert.equal(index.clips.filter(c=>c.status==='CONTROL').length,4);
 assert.equal(index.clips.filter(c=>c.single_support_1s).length,0);
 for(const c of index.clips.filter(c=>c.group==='walk'))assert.deepEqual(c.support_intervals_s,{left:.12,right:.14});
 for(const mutate of [d=>d.clips[13]=d.clips[0],d=>d.clips[0].url='/assets/../private.bin',d=>d.clips[0].status='PASS',d=>d.clips[1].observations.pop(),d=>d.clips[2].terrain.height_m=.05]){
  const d=structuredClone(index);mutate(d);assert.throws(()=>validateIndex(d,sha));
 }
});
test('all 5540 source-bound frames decode as 70 original mesh poses',async()=>{
 let total=0;
 for(const c of index.clips){const v=await decodePoseBytes(buffer(c),c,index);total+=v.frames.length;
  assert.equal(v.frames.length,c.frames);assert.equal(v.frames[0].length,498);assert.ok(Math.abs(v.frames.at(-1)[0]-c.duration_s)<1e-5);}
 assert.equal(total,5540);
});
test('hash corruption and truncated data are rejected',async()=>{
 const c=index.clips[0];let b=buffer(c);new Uint8Array(b)[100]^=1;
 await assert.rejects(()=>decodePoseBytes(b,c,index),/сумма/);
 await assert.rejects(()=>decodePoseBytes(b.slice(0,-4),c,index),/размер/);

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
 const entry=catalog.entries.find(e=>e.id==='microduck-single-support-v1');
 assert.equal(entry.files.length,19);
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
 assert.ok(requests.filter(url=>url.includes('/docs/hexapod/microduck-single-support-v1/')).length>=19);
 assert.ok(requests.every(url=>!url.includes('/assets/robot-replays/')));
});
