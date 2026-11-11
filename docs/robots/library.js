// Universal robot library: legacy research routes plus standalone robot-replay/v1 files.
// GitHub main is authoritative. Both manifests are fetched from the SAME immutable commit.
import {validateCatalog,readBoundedResponse} from '../replay/catalog.js';

const SCHEMA='alfred-robot-library/v1';
const API='https://api.github.com/repos/Dmitry-dev-pet/alfred-hexapod-lab/branches/main';
const RAW='https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/';
const KEY='alfred-robot-unified-main-v1';
const ID=/^[a-z0-9-]{1,64}$/;
const LEGACY=/^\/hexapod\/(?:[a-z0-9-]+\/)?$/;
const SHA=/^[a-f0-9]{40}$/;

export function robotFamily(id) {
  const x=id.toLowerCase();
  if(x.includes('hexarth'))return 'HexArth';
  if(x.includes('bb8'))return 'BB-8';
  if(x.includes('microduck'))return 'MicroDuck';
  if(x.includes('en01'))return 'NavBot EN01';
  if(x.includes('pupper'))return 'Stanford Pupper';
  if(x.includes('lite3'))return 'XGO Lite 3';
  if(x.includes('xgorider'))return 'XGO Rider2';
  if(x.includes('wheelbot'))return 'Mini Wheelbot';
  if(x.includes('sebaju'))return 'SeBaJu';
  if(x.includes('beni'))return 'OpenBeni';
  if(x.includes('cheetah'))return 'Mini Cheetah';
  return 'Freenove FNK0052';
}

export function validateLegacyIndex(doc) {
  if(doc?.schema!==SCHEMA||!Number.isInteger(doc.revision)||doc.revision<1||
     doc.replay_catalog_path!=='docs/replay/catalog.json'||
     !Array.isArray(doc.entries)||doc.entries.length>500)
    throw Error('Неверный общий каталог роботов');
  const ids=new Set();
  for(const e of doc.entries) {
    if(!ID.test(e.id)||ids.has(e.id)||e.kind!=='legacy'||
       !LEGACY.test(e.href)||typeof e.robot!=='string'||!e.robot||e.robot.length>70||
       typeof e.title!=='string'||!e.title||e.title.length>120||
       typeof e.detail!=='string'||e.detail.length>300||
       !Number.isFinite(e.duration_s)||e.duration_s<=0||
       !['published','pending'].includes(e.availability))
      throw Error('Неверная историческая запись');
    ids.add(e.id);
  }
  return doc;
}

export function joinRobotCatalogs(legacy, recordingCatalog) {
  validateLegacyIndex(legacy);
  validateCatalog(recordingCatalog);
  const ids=new Set(legacy.entries.map(e=>e.id));
  for(const e of recordingCatalog.entries) {
    if(ids.has(e.id))throw Error('Повтор идентификатора между каталогами: '+e.id);
    ids.add(e.id);
  }
  const recordings=recordingCatalog.entries.map(e=>({
    id:e.id,robot:robotFamily(e.id),kind:'recording',
    title:e.title,detail:e.detail,duration_s:e.duration_s,
    href:'/replay/?recording='+encodeURIComponent(e.id),
    availability:'published',result_status:e.status||'research',recording:e.recording
  }));
  const entries=[...recordings,...legacy.entries.map(e=>({...e}))];
  return {schema:SCHEMA,revision:legacy.revision,replay_revision:recordingCatalog.revision,
    entries,source:'github-main',
    counts:{total:entries.length,published:entries.filter(e=>e.availability==='published').length,
      pending:entries.filter(e=>e.availability==='pending').length,
      recordings:recordings.length,legacy:legacy.entries.length}};
}

function siteIndex(legacy) {
  // Offline compatibility with the previous Site manifest. No extra HTML/JS is fetched.
  if(legacy?.schema!=='hexapod-library/v1'||!Array.isArray(legacy.entries)||legacy.entries.length>500)
    throw Error('Неверная старая библиотека');
  return validateLegacyIndex({schema:SCHEMA,revision:Number.isInteger(legacy.revision)&&legacy.revision>0?legacy.revision:1,
    replay_catalog_path:'docs/replay/catalog.json',
    entries:legacy.entries.map(e=>({id:e.id,robot:robotFamily(e.id),kind:'legacy',
      title:e.title,detail:e.detail||'',duration_s:e.duration_s,href:e.href,
      availability:'published',...(e.status?{result_status:e.status}:{})}))});
}

async function requestJSON(fetchImpl,url,signal) {
  const response=await fetchImpl(url,{cache:'no-store',credentials:'omit',redirect:'error',
    signal:signal||AbortSignal.timeout(12000)});
  const bytes=await readBoundedResponse(response,300000);
  return JSON.parse(new TextDecoder().decode(bytes));
}

export async function readRobotLibrary({fetchImpl=fetch,storage=globalThis.localStorage,signal}={}) {
  let cached=null;
  try {
    const doc=JSON.parse(storage?.getItem(KEY)||'null');
    if(doc?.schema===SCHEMA&&Array.isArray(doc.entries)&&doc.entries.length<=800&&
       Number.isInteger(doc.revision)&&doc.entries.every(e=>ID.test(e.id)))cached=doc;
  }catch{}
  const keep=doc=>{try{storage?.setItem(KEY,JSON.stringify(doc));}catch{}return doc;};
  try {
    const branch=await requestJSON(fetchImpl,API,signal);
    const sha=branch?.commit?.sha;
    if(!SHA.test(sha))throw Error('Неверный SHA ветки main');
    const [legacy,recordings]=await Promise.all([
      requestJSON(fetchImpl,RAW+sha+'/docs/robots/catalog.json',signal),
      requestJSON(fetchImpl,RAW+sha+'/docs/replay/catalog.json',signal)
    ]);
    const result=joinRobotCatalogs(legacy,recordings);
    result.source='github-main@'+sha;
    return keep(result);
  }catch(remoteError) {
    if(cached)return {...cached,source:'offline-cache'};
    try {
      const [legacy,recordings]=await Promise.all([
        requestJSON(fetchImpl,'/robots/catalog.json',signal).catch(()=>
          requestJSON(fetchImpl,'/hexapod/library.json',signal).then(siteIndex)),
        requestJSON(fetchImpl,'/replay/catalog.json',signal)
      ]);
      const result=joinRobotCatalogs(legacy,recordings);
      result.source='site-fallback';
      return keep(result);
    }catch {
      throw Error('Единый каталог недоступен: '+remoteError.message);
    }
  }
}
