import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import {validateLegacyIndex,joinRobotCatalogs,readRobotLibrary} from '../docs/robots/library.js';

const index=JSON.parse(await fs.readFile(new URL('../docs/robots/catalog.json',import.meta.url),'utf8'));
const replays=JSON.parse(await fs.readFile(new URL('../docs/replay/catalog.json',import.meta.url),'utf8'));
const all=joinRobotCatalogs(index,replays);

assert.equal(index.replay_catalog_path,'docs/replay/catalog.json');
assert.equal(index.entries.length,79);
assert.equal(all.counts.legacy,79);
assert.equal(all.counts.recordings,replays.entries.length);
assert.equal(all.counts.total,index.entries.length+replays.entries.length);
assert.equal(all.counts.pending,4);
assert.equal(all.counts.published,all.counts.total-4);
assert.equal(new Set(all.entries.map(e=>e.id)).size,all.entries.length);
assert.equal(new Set(all.entries.map(e=>e.robot)).size,12);
for(const e of all.entries){
  assert.match(e.id,/^[a-z0-9-]+$/);
  if(e.kind==='recording')assert.equal(e.href,'/replay/?recording='+e.id);
  else assert.match(e.href,/^\/hexapod\/(?:[a-z0-9-]+\/)?$/);
}
assert.deepEqual(index.entries.filter(e=>e.availability==='pending').map(e=>e.id).sort(),
  ['microduck-step-goal-v1','microduck-single-support-v1','openbeni-recovery-v1','openbeni-colored-v1'].sort());
const bad=structuredClone(index);bad.entries[0].href='javascript:alert(1)';
assert.throws(()=>validateLegacyIndex(bad),/историческая/);
const dup=structuredClone(index);dup.entries[0].id=replays.entries[0].id;
assert.throws(()=>joinRobotCatalogs(dup,replays),/Повтор идентификатора/);

const sha='a'.repeat(40);
const requests=[];
const fetchImpl=async url=>{
  requests.push(String(url));
  if(String(url).endsWith('/branches/main'))return Response.json({commit:{sha}});
  if(String(url).endsWith('/'+sha+'/docs/robots/catalog.json'))return Response.json(index);
  if(String(url).endsWith('/'+sha+'/docs/replay/catalog.json'))return Response.json(replays);
  throw Error('Unexpected URL '+url);
};
const values=new Map(),storage={getItem:key=>values.get(key),setItem:(key,value)=>values.set(key,value)};
const online=await readRobotLibrary({fetchImpl,storage});
assert.deepEqual(online.counts,all.counts);
assert.equal(online.source,'github-main@'+sha);
assert.equal(requests.length,3);
assert.ok(requests[1].includes('/'+sha+'/'));
assert.ok(requests[2].includes('/'+sha+'/'));
const offline=await readRobotLibrary({fetchImpl:async()=>{throw Error('offline');},storage});
assert.equal(offline.source,'offline-cache');
assert.deepEqual(offline.counts,all.counts);
console.log('PASS: unified catalog, 12 robot families, 79 legacy + '+replays.entries.length+
  ' recordings, four pending, immutable main fetch, duplicate/scheme rejection, offline cache');
