import test from 'node:test';
import assert from 'node:assert/strict';
import {gzipSync} from 'node:zlib';
import {createHash,webcrypto} from 'node:crypto';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source=await fs.readFile(new URL('../docs/hexapod/model-package.js',import.meta.url),'utf8');
const hash=b=>createHash('sha256').update(b).digest('hex');
function fixture(){
  const stl=Buffer.alloc(84);stl.write('original geometry');
  const pack={schema:'robot-model-package/v1',id:'test-robot',encoding:'gzip-stl-base64',
    manifest:{revision:'a'.repeat(40),meshes:{part:{file:'part.stl',size:stl.length,sha256:hash(stl)}},visible_geoms:[{mesh:'part'},{mesh:'part'}]},
    meshes:{part:gzipSync(stl).toString('base64')},license:{notice:'Fixture licence',spdx:'CC0-1.0'}};
  const bytes=Buffer.from(JSON.stringify(pack));
  const ref={schema:'robot-model-package-ref/v1',id:pack.id,url:'/assets/robot-models/'+pack.id+'-'+hash(bytes)+'.json',sha256:hash(bytes),size:bytes.length,mesh_count:1,instance_count:2,source_revision:'a'.repeat(40),license_spdx:'CC0-1.0'};
  return {pack,bytes,ref,stl};
}
async function setup(fetch){
  const stored=new Map();
  const context=vm.createContext({crypto:webcrypto,Uint8Array,TextDecoder,Blob,Response,DecompressionStream,atob,fetch,
    caches:{open:async()=>({match:async k=>stored.get(k)?.clone(),delete:async k=>stored.delete(k),put:async(k,v)=>stored.set(k,v.clone())})}});
  const module=new vm.SourceTextModule(source,{context});await module.link(()=>{});await module.evaluate();
  return {api:module.namespace,stored};
}
test('one request for concurrent/repeated model uses; shared STL remains separate instances',async()=>{
  const f=fixture();let calls=0;
  const {api,stored}=await setup(async()=>{calls++;return new Response(f.bytes);});
  const [a,b]=await Promise.all([api.loadModelPackage(f.ref),api.loadModelPackage(f.ref)]);
  assert.equal(calls,1);assert.equal(a,b);assert.deepEqual(Buffer.from(a.meshes.get('part')),f.stl);
  assert.equal(a.manifest.visible_geoms.length,2);assert.equal(stored.size,1);
  await api.loadModelPackage(f.ref);assert.equal(calls,1);
});
test('new page reuses verified browser cache without network',async()=>{
  const f=fixture();const {api,stored}=await setup(async()=>{throw Error('offline');});
  stored.set(f.ref.url,new Response(f.bytes));
  assert.equal((await api.loadModelPackage(f.ref)).meshes.size,1);
});
test('corrupt cached package is discarded, replaced and checked',async()=>{
  const f=fixture();let calls=0;const {api,stored}=await setup(async()=>{calls++;return new Response(f.bytes);});
  stored.set(f.ref.url,new Response('broken'));await api.loadModelPackage(f.ref);
  assert.equal(calls,1);assert.equal((await stored.get(f.ref.url).arrayBuffer()).byteLength,f.bytes.length);
});
test('corrupt network response fails closed and does not poison retry',async()=>{
  const f=fixture();let calls=0;const {api,stored}=await setup(async()=>new Response(++calls===1?'broken':f.bytes));
  await assert.rejects(api.loadModelPackage(f.ref),/сумма/);assert.equal(stored.size,0);
  assert.equal((await api.loadModelPackage(f.ref)).meshes.size,1);assert.equal(calls,2);
});
test('valid outer hash cannot conceal a changed original STL digest',async()=>{
  const f=fixture();f.pack.manifest.meshes.part.sha256='0'.repeat(64);
  const bytes=Buffer.from(JSON.stringify(f.pack));const sha=hash(bytes);
  const ref={...f.ref,size:bytes.length,sha256:sha,url:'/assets/robot-models/test-robot-'+sha+'.json'};
  const {api}=await setup(async()=>new Response(bytes));
  await assert.rejects(api.loadModelPackage(ref),/STL/);
});
test('declared small STL rejects oversized gzip expansion',async()=>{
  const f=fixture();f.pack.meshes.part=gzipSync(Buffer.alloc(200)).toString('base64');
  const bytes=Buffer.from(JSON.stringify(f.pack));const sha=hash(bytes);
  const ref={...f.ref,size:bytes.length,sha256:sha,url:'/assets/robot-models/test-robot-'+sha+'.json'};
  const {api}=await setup(async()=>new Response(bytes));
  await assert.rejects(api.loadModelPackage(ref),/размер/);
});
test('unsafe URL is rejected before fetching',async()=>{
  const f=fixture();const {api}=await setup(async()=>{throw Error('must not fetch');});
  await assert.rejects(api.loadModelPackage({...f.ref,url:'https://untrusted.invalid/model.json'}),/ссылка/);
});
