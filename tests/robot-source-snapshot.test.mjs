import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url));
const read=async p=>JSON.parse(await fs.readFile(path.join(root,p),'utf8'));
const old=await read('docs/hexapod/library.json');
const global=await read('docs/robots/catalog.json');
const replay=await read('docs/replay/catalog.json');
const site=await read('docs/robots/site-only-file-manifests.json');
assert.equal(old.schema,'hexapod-library/v1');
assert.equal(old.revision,61);
assert.equal(old.entries.length,77);
assert.equal(global.schema,'alfred-robot-library/v1');
assert.equal(global.entries.length,79);
assert.equal(replay.entries.length,3);
assert.equal(site.records.length,2);
assert.equal(global.entries.length+replay.entries.length,82);
const allIDs=new Set([...global.entries,...replay.entries].map(e=>e.id));
assert.equal(allIDs.size,82);
const canonical=new Map(global.entries.map(e=>[e.id,e]));
for(const e of old.entries){
  assert.equal(canonical.get(e.id)?.href,e.href,'Legacy href retained '+e.id);
}
for(const e of site.records){
  assert.equal(canonical.get(e.id)?.href,e.href,'Published Site-only href retained '+e.id);
  assert(e.files.length>0);
}
assert.deepEqual(global.entries.filter(e=>e.availability==='pending').map(e=>e.id).sort(),
  ['microduck-step-goal-v1','microduck-single-support-v1',
   'openbeni-recovery-v1','openbeni-colored-v1'].sort());
let count=0,total=0;
for(const entry of old.entries)for(const file of entry.files){
  assert.match(file.path,/^hexapod\/(?!.*\.\.)/);
  assert.match(file.sha256,/^[0-9a-f]{64}$/);
  const bytes=await fs.readFile(path.join(root,'docs',file.path));
  assert.equal(bytes.length,file.size,'Size mismatch '+file.path);
  assert.equal(createHash('sha256').update(bytes).digest('hex'),file.sha256,
    'SHA-256 mismatch '+file.path);
  total+=bytes.length;count++;
}
assert.equal(count,334);
assert.equal(total,89288219);
console.log(JSON.stringify({status:'PASS',old_cards:77,site_only:2,new_replays:3,
  total_cards:82,verified_assets:count,verified_bytes:total}));
