import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp, mkdir, writeFile, rm, copyFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
const script=fileURLToPath(new URL('../scripts/check_recording_separation.py',import.meta.url));
test('publication accepts metadata and rejects every bundled motion location',async()=>{
 const root=await mkdtemp(path.join(tmpdir(),'robot-separation-'));
 try {
  await mkdir(path.join(root,'dist/replay'),{recursive:true});
  await mkdir(path.join(root,'dist/robots'),{recursive:true});
  try{await copyFile(new URL('../dist/replay/catalog.json',import.meta.url),path.join(root,'dist/replay/catalog.json'));}
  catch{await copyFile(new URL('../docs/replay/catalog.json',import.meta.url),path.join(root,'dist/replay/catalog.json'));}
  try{await copyFile(new URL('../dist/robots/recording-assets.json',import.meta.url),path.join(root,'dist/robots/recording-assets.json'));}
  catch{await copyFile(new URL('../docs/robots/recording-assets.json',import.meta.url),path.join(root,'dist/robots/recording-assets.json'));}
  const run=()=>spawnSync('python3',[script,'--root',root],{encoding:'utf8'});
  assert.equal(run().status,0);
  for(const relative of ['replay/fixture.robotreplay','assets/robot-replays/test.bin','assets/robot-motions/test.json','hexapod/data/test.json']){
   const target=path.join(root,'dist',relative);
   await mkdir(path.dirname(target),{recursive:true});await writeFile(target,'test');
   assert.notEqual(run().status,0,relative+' must block publication');
   await rm(target);
  }
 }finally{await rm(root,{recursive:true,force:true});}
});
