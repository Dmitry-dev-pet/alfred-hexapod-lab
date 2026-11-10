import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import {validateCatalog,libraryEntries,readCatalog,fetchRecording,readBoundedResponse} from '../docs/replay/catalog.js';
import {loadReplay} from '../docs/replay/format.js';
import {createHash} from 'node:crypto';
const catalog=JSON.parse(await fs.readFile(new URL('../docs/replay/catalog.json',import.meta.url)));
const [entry]=libraryEntries(catalog);assert.equal(entry.href,'/replay/?recording=hexarth-four-experiments-v1');
let raw,hasPackage=true;try{raw=await fs.readFile(new URL('../dist'+entry.recording.url,import.meta.url));}catch(e){if(e.code!=='ENOENT')throw e;raw=Buffer.from('bounded catalog transport fixture');hasPackage=false;}
const reference=hasPackage?entry:{...entry,recording:{...entry.recording,size:raw.length,sha256:createHash('sha256').update(raw).digest('hex')}};let requests=[];
const fetched=await fetchRecording(reference,{base:'https://player.test/',fetchImpl:async(url,options)=>{requests.push({url:String(url),credentials:options.credentials});return new Response(raw);}});
assert.deepEqual(Buffer.from(fetched),raw);assert.equal(requests.length,1);assert.equal(requests[0].credentials,entry.recording.url.startsWith('https://')?'omit':'same-origin');
if(hasPackage){const replay=await loadReplay(fetched);assert.equal(replay.clips.length,4);assert.equal(replay.meshes.length,19);}
await assert.rejects(fetchRecording({...reference,recording:{...reference.recording,sha256:'0'.repeat(64)}},{fetchImpl:async()=>new Response(raw)}),/Контрольная сумма/);
await assert.rejects(readBoundedResponse(new Response(new Uint8Array(10)),9),/слишком большой/);
for(const change of [d=>d.entries[0].recording.url='javascript:alert(1)',d=>d.entries[0].recording.url='//other.test/file',d=>d.entries.push(d.entries[0]),d=>d.entries[0].recording.size=0]){const bad=structuredClone(catalog);change(bad);assert.throws(()=>validateCatalog(bad));}
const saved=new Map();const storage={getItem:k=>saved.get(k),setItem:(k,v)=>saved.set(k,v)};
const chosen=await readCatalog({storage,fetchImpl:async url=>{if(url==='/replay/catalog.json')return Response.json(catalog);throw Error('offline');}});assert.equal(chosen.entries[0].id,entry.id);
const offline=await readCatalog({storage,fetchImpl:async()=>{throw Error('offline');}});assert.equal(offline.entries[0].id,entry.id);
console.log('PASS: automatic catalog discovery → one verified file request; hash/size/unsafe URLs/duplicates rejected; offline catalog retained; '+(hasPackage?'real four-clip package decoded':'large package not present; transport fixture only'));
