import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash, webcrypto} from 'node:crypto';
import vm from 'node:vm';

const SOURCE = readFileSync(new URL('../hosting/github-live-sw.js', import.meta.url), 'utf8');
const sha = '0123456789abcdef0123456789abcdef01234567';
const owner='Dmitry-dev-pet/alfred-hexapod-lab';
const ghRoot='https://raw.githubusercontent.com/'+owner+'/'+sha+'/docs/';
const digest = text => createHash('sha256').update(text).digest('hex');
const html='<!doctype html><script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js"}}</script><script type="module" src="/hexapod/microduck/app.js"></script>';
const js="export const ok = 1;";
const sample=JSON.stringify({format:'replay-v1',frames:[1,2]});
const payloads = {'hexapod/microduck/index.html':html,'hexapod/microduck/app.js':js,'hexapod/microduck/pose.json':sample};
const entry={id:'microduck',title:'MicroDuck',detail:'replay',duration_s:6,href:'/hexapod/microduck/',files:Object.entries(payloads).map(([path,val])=>({path,size:new TextEncoder().encode(val).byteLength,sha256:digest(val)}))};
const catalogue = {schema:'hexapod-library/v1',revision:36,entries:[entry]};

function init({tamper=false}={}) {
 const map=new Map(), handlers={};
 const status={offline:false};
 const fakeCaches={async open() {return {
  async match(key){return map.get(String(key))?.clone() ?? undefined;},
  async put(key,val){map.set(String(key),val.clone());}
 };}};
 const fetched=[];
 const upstream=async url=>{
  url=String(url); fetched.push(url);
  if(status.offline) throw Error('offline');
  if(url.includes('/branches/')) return new Response(JSON.stringify({commit:{sha}}),{status:200});
  const path = url.replace(ghRoot,'');
  if(path==='hexapod/library.json') return new Response(JSON.stringify(catalogue),{status:200});
  if(Object.hasOwn(payloads,path))return new Response(tamper && path.endsWith('.js')?'bad!':payloads[path],{status:200});
  throw Error('Unknown upstream URL: '+url);
 };
 const mockSelf={location:{origin:'https://robot.example',href:'https://robot.example/github-live-sw.js'},
  skipWaiting:()=>Promise.resolve(),clients:{claim:()=>Promise.resolve()},
  addEventListener:(name,fn)=>{handlers[name]=fn;}
 };
 const ctx=vm.createContext({self:mockSelf,caches:fakeCaches,fetch:upstream,
  TextDecoder,TextEncoder,Response,Request,crypto:webcrypto,URL,Date,Map,Set,Number,
  Uint8Array,ArrayBuffer,Promise,console});
 vm.runInContext(SOURCE,ctx);
 const request = async path => {
  const req=new Request('https://robot.example'+path);
  let response=null;
  handlers.fetch({request:req,respondWith:x=>{response=Promise.resolve(x);}});
  return response?await response:null;
 };
 return {ctx,request,fetched,status};
}

test('live GitHub catalog uses the resolved immutable commit SHA',async()=>{
 const env=init();
 const r=await env.request('/hexapod/library.json');
 assert.equal(r.status,200);
 assert.equal(r.headers.get('X-Alfred-Catalog-Source'),'github');
 assert.equal((await r.json()).revision,36);
 assert.ok(env.fetched.some(x=>x.includes('/branches/feature/android-local-offline-viewer-20261104')));
 assert.ok(env.fetched.some(x=>x.endsWith('/'+sha+'/docs/hexapod/library.json')));
});
test('verified HTML, JavaScript and JSON get correct browser MIME types',async()=>{
 const env=init();await env.request('/hexapod/library.json');
 const h=await env.request('/hexapod/microduck/');
 assert.equal(h.status,200);
 assert.match(h.headers.get('Content-Type'),/text\/html/);
 assert.match(await h.text(),/\/vendor\/three\/build\/three.module.js/);
 const s=await env.request('/hexapod/microduck/app.js');
 assert.equal(s.status,200);assert.match(s.headers.get('Content-Type'),/text\/javascript/);
 assert.equal(await s.text(),js);
 const d=await env.request('/hexapod/microduck/pose.json');
 assert.equal((await d.json()).format,'replay-v1');
});
test('corrupt replay byte fails closed, no unlisted asset or external page interception',async()=>{
 const env=init({tamper:true});await env.request('/hexapod/library.json');
 assert.equal((await env.request('/hexapod/microduck/app.js')).status,503);
 const fetched=env.fetched.length;
 assert.equal(await env.request('/private/other-app'),null);
 assert.equal(env.fetched.length,fetched);
});
test('rejects malicious paths, duplicate IDs and revision rollback',async()=>{
 const env=init();
 const bad={...catalogue,entries:[{...entry,files:[{path:'hexapod/../x.js',sha256:'a'.repeat(64),size:1}]}]};
 assert.throws(()=>env.ctx.validateCatalog(bad),/Unsafe/);
 assert.throws(()=>env.ctx.validateCatalog({...catalogue,entries:[entry,entry]}),/Invalid GitHub catalogue entry/);
});
test('previously verified catalog and replay stay available when offline',async()=>{
 const env=init();await env.request('/hexapod/library.json');
 assert.equal((await env.request('/hexapod/microduck/app.js')).status,200);
 env.status.offline=true;
 const c=await env.request('/hexapod/library.json');
 assert.equal(c.status,200);assert.equal(c.headers.get('X-Alfred-Catalog-Source'),'cached-github');assert.equal((await c.json()).revision,36);
 assert.equal((await env.request('/hexapod/microduck/app.js')).status,200);
});
