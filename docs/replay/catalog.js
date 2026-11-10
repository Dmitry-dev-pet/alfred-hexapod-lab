// Data-only catalog. All scene code stays in the installed player.
import {sha256} from './format.js';
const ID=/^[a-z0-9-]{1,64}$/, DIGEST=/^[a-f0-9]{64}$/;
const MAX=100*1024*1024, CACHE='alfred-single-file-catalog-v1';
const API='https://api.github.com/repos/Dmitry-dev-pet/alfred-hexapod-lab/branches/feature/android-local-offline-viewer-20261104';
const RAW='https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/';
export function validateCatalog(doc) {
  if(doc?.schema!=='robot-replay-catalog/v1'||!Number.isInteger(doc.revision)||doc.revision<1||!Array.isArray(doc.entries)||doc.entries.length>300)throw Error('Неверный каталог записей');
  const ids=new Set();
  for(const e of doc.entries){
    if(!ID.test(e.id)||ids.has(e.id)||typeof e.title!=='string'||e.title.length>100||typeof e.detail!=='string'||e.detail.length>180||!Number.isFinite(e.duration_s)||e.duration_s<=0||e.recording?.schema!=='robot-replay/v1'||!DIGEST.test(e.recording.sha256)||!Number.isInteger(e.recording.size)||e.recording.size<1||e.recording.size>MAX||typeof e.recording.url!=='string')throw Error('Неверная запись каталога');
    const url=new URL(e.recording.url,'https://player.invalid');
    if(url.protocol!=='https:'||url.username||url.password||url.hash||e.recording.url.startsWith('//')||(!e.recording.url.startsWith('/')&&!e.recording.url.startsWith('https://')))throw Error('Небезопасный адрес записи');
    ids.add(e.id);
  }
  return doc;
}
export function libraryEntries(doc){return validateCatalog(doc).entries.map(e=>({...e,href:'/replay/?recording='+encodeURIComponent(e.id)}));}
export async function readBoundedResponse(response,limit=MAX){
  if(!response.ok)throw Error('HTTP '+response.status);
  if(Number(response.headers.get('content-length'))>limit)throw Error('Файл слишком большой');
  if(!response.body){const bytes=await response.arrayBuffer();if(bytes.byteLength>limit)throw Error('Файл слишком большой');return bytes;}
  const reader=response.body.getReader(),chunks=[];let count=0;
  try{for(;;){const {value,done}=await reader.read();if(done)break;count+=value.length;if(count>limit)throw Error('Файл слишком большой');chunks.push(value);}}
  catch(e){await reader.cancel().catch(()=>{});throw e;}
  const out=new Uint8Array(count);let offset=0;for(const chunk of chunks){out.set(chunk,offset);offset+=chunk.length;}return out.buffer;
}
export async function fetchRecording(entry,{fetchImpl=fetch,base=globalThis.location?.href||'https://player.invalid/'}={}){
  validateCatalog({schema:'robot-replay-catalog/v1',revision:1,entries:[entry]});
  const r=await fetchImpl(new URL(entry.recording.url,base),{credentials:'same-origin',redirect:'error',signal:AbortSignal.timeout(60000)});
  const bytes=await readBoundedResponse(r,entry.recording.size);
  if(bytes.byteLength!==entry.recording.size||await sha256(bytes)!==entry.recording.sha256)throw Error('Контрольная сумма записи не совпадает');
  return bytes;
}
export async function readCatalog({fetchImpl=fetch,storage=globalThis.localStorage,signal}={}){
  const json=async(url)=>JSON.parse(new TextDecoder().decode(await readBoundedResponse(await fetchImpl(url,{cache:'no-store',credentials:'same-origin',signal:signal||AbortSignal.timeout(8000)}),200000)));
  let local=null,cached=null;
  try{cached=validateCatalog(JSON.parse(storage?.getItem(CACHE)||'null'));}catch{}
  try{local=validateCatalog(await json('/replay/catalog.json'));}catch{}
  // A bundled usable catalog opens instantly; refresh the external catalog without
  // blocking the library. New data descriptors never require per-recording JS.
  const remote=async()=>{const head=await json(API),sha=head?.commit?.sha;if(!/^[a-f0-9]{40}$/.test(sha))throw Error('Неверная ревизия каталога');return validateCatalog(await json(RAW+sha+'/docs/replay/catalog.json'));};
  const remember=doc=>{try{storage?.setItem(CACHE,JSON.stringify(doc));}catch{}return doc;};
  if(local?.entries.length){const chosen=cached&&cached.revision>local.revision?cached:local;remote().then(doc=>{if(doc.revision>chosen.revision){remember(doc);globalThis.dispatchEvent?.(new CustomEvent('robot-recording-catalog',{detail:doc}));}}).catch(()=>{});return remember(chosen);}
  try{const doc=await remote();if(cached&&cached.revision>doc.revision)return cached;return remember(doc);}catch{if(cached)return cached;if(local)return local;throw Error('Каталог записей недоступен');}
}
