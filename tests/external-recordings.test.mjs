import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import {readCatalog,fetchRecording} from '../docs/replay/catalog.js';
import worker,{validManifest} from '../scripts/recording-worker.mjs';
const original=JSON.parse(await fs.readFile(new URL('../docs/replay/catalog.json',import.meta.url)));
const saved=new Map(),storage={getItem:k=>saved.get(k),setItem:(k,v)=>saved.set(k,v)};
let remote=structuredClone(original);remote.revision=4;
const sha='1'.repeat(40);
const fetchImpl=async url=>String(url).includes('/branches/')?Response.json({commit:{sha}}):
 String(url).startsWith('https://raw.githubusercontent.com/')?Response.json(remote):Response.json(original);
assert.equal((await readCatalog({fetchImpl,storage})).revision,4,'online GitHub catalog wins over the bundled snapshot');
remote=structuredClone(remote);remote.revision=5;remote.entries.push({...remote.entries[0],id:'new-recording-with-unchanged-site',title:'A later real recording descriptor'});
assert.equal((await readCatalog({fetchImpl,storage})).entries.length,original.entries.length+1,'unchanged reader discovers a later catalog entry');
assert.equal((await readCatalog({fetchImpl:async()=>{throw Error('offline');},storage})).revision,5,'last verified catalog survives offline');
let credentials;
await assert.rejects(fetchRecording(original.entries[0],{base:'https://player.test/',fetchImpl:async(u,o)=>{credentials=o.credentials;return new Response(new Uint8Array());}}));
assert.equal(credentials,'omit','external file requests carry no Site credentials');
const hash='a'.repeat(64),manifest={schema:'robot-recording-object/v1',sha256:hash,size:3,chunks:[{key:'chunk-'+hash+'-0',size:3}]};
assert(validManifest(manifest,hash));assert(!validManifest({...manifest,chunks:[{key:'arbitrary-key',size:3}]},hash));
const kv={get:async(k,type)=>k.startsWith('manifest-')?manifest:new Uint8Array([1,2,3]).buffer};
const response=await worker.fetch(new Request('https://files.test/records/'+hash+'.robotreplay'),{RECORDINGS:kv});
assert.equal(response.headers.get('access-control-allow-origin'),'*');assert.deepEqual(new Uint8Array(await response.arrayBuffer()),new Uint8Array([1,2,3]));
assert.equal((await worker.fetch(new Request('https://files.test/records/'+hash+'.robotreplay',{method:'PUT'}),{RECORDINGS:kv})).status,405,'public file service is read only');
assert.equal((await worker.fetch(new Request('https://files.test/not-a-recording'),{RECORDINGS:kv})).status,404);
async function check(dir){for(const d of await fs.readdir(dir,{withFileTypes:true})){const p=dir+'/'+d.name;if(d.isDirectory())await check(p);else assert(!d.name.endsWith('.robotreplay'),'Site must not bundle recording payloads');}}
await check(new URL('../docs',import.meta.url).pathname);
console.log('PASS: catalog-only additions, online freshness, offline fallback, credential-free external reads, immutable chunk service, no bundled recordings');
