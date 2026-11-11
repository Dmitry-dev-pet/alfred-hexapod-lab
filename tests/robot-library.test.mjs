import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {mergeCatalogs,modelFor,readUnifiedLibrary} from '../docs/robot-library/catalog.js';

const root=fileURLToPath(new URL('../', import.meta.url));
const read=async p=>JSON.parse(await fs.readFile(path.join(root,p),'utf8'));
const old=await read('docs/hexapod/library.json');
const replay=await read('docs/replay/catalog.json');
const extra=await read('docs/robot-library/site-only-legacy.json');
const combined=mergeCatalogs(old,replay,extra);
assert.equal(old.revision,61);
assert.equal(combined.schema,'alfred-robot-library/v1');
assert.deepEqual(combined.counts,{legacy:77,site_only:2,replay:3,total:82});
assert.equal(new Set(combined.entries.map(e=>e.id)).size,82);
assert.equal(new Set(combined.entries.map(e=>e.href)).size,82);
assert.equal(combined.entries.filter(e=>e.site_status==='not-published').length,4);
assert(combined.entries.some(e=>e.id==='en01-physics-torque-v9'&&e.source==='legacy-site'));
assert(combined.entries.some(e=>e.id==='en01-tuck-v8-recordings-20261109'&&e.source==='legacy-site'));
assert(combined.entries.some(e=>e.id==='openbeni-colored-v1'&&e.site_status==='not-published'));
assert(combined.entries.some(e=>e.id==='freenove-stand-torso-2s'&&e.format==='robot-replay/v1'));
assert.equal(modelFor('contact-four'),'freenove');
assert.equal(modelFor('hexarth-four-experiments-v1'),'hexarth');

// Hash every asset listed in the historical source catalog, not just titles.
let files=0,bytes=0;
for(const e of old.entries)for(const file of e.files){
  const body=await fs.readFile(path.join(root,'docs',file.path));
  assert.equal(body.byteLength,file.size, file.path + ' size');
  assert.equal(createHash('sha256').update(body).digest('hex'),file.sha256,file.path + ' digest');
  files++;bytes+=body.byteLength;
}
assert.equal(files,334);
const sha='a'.repeat(40);
const inputs=[
  ['docs/hexapod/library.json',old],
  ['docs/replay/catalog.json',replay],
  ['docs/robot-library/site-only-legacy.json',extra]
];
const requests=[];
const remote=await readUnifiedLibrary({fetchImpl:async url=>{
  requests.push(url);
  const obj=String(url).endsWith('/branches/main')?{commit:{sha}}:
    inputs.find(([name])=>String(url).endsWith('/'+sha+'/'+name))?.[1];
  if(!obj)throw Error('Unexpected request '+url);
  return {ok:true,headers:{get:()=>null},text:async()=>JSON.stringify(obj)};
}});
assert.equal(remote.source_commit,sha);
assert.equal(remote.entries.length,82);
assert.equal(requests.length,4);
assert(requests.slice(1).every(url=>url.includes('/'+sha+'/')));
for(const change of [
  d=>d.entries.push(d.entries[0]),
  d=>d.entries[0].href='javascript:alert(1)',
  d=>d.entries[0].files[0].sha256='nope'
]){
  const tampered=structuredClone(old);change(tampered);
  assert.throws(()=>mergeCatalogs(tampered,replay,extra));
}
console.log(JSON.stringify({status:'PASS',catalogs:combined.counts,verified_files:files,verified_bytes:bytes,remote:'GitHub main pinned SHA'}));
