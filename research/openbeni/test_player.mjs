import fs from 'node:fs/promises';
import path from 'node:path';
const dir=path.resolve(process.argv[2]??'docs/hexapod/openbeni-recovery-v1');
const {validateIndex,loadVerified,decodeRecord}=await import('file://'+dir+'/record-loader.js');
globalThis.fetch=async url=>new Response(await fs.readFile(path.join(dir,url)));
const index=validateIndex(JSON.parse(await fs.readFile(dir+'/replays.json')));
let frames=0;
for(const clip of index.clips){
  const record=await loadVerified(clip.url,clip.size,clip.sha256,b=>decodeRecord(b,clip,index));
  frames+=record.frame_count;
}
let corrupt=false;
try{await loadVerified(index.clips[0].url,index.clips[0].size,'0'.repeat(64),b=>b);}catch{corrupt=true;}
let unsafe=false;
try{validateIndex({...index,clips:[{...index.clips[0],url:'../other'},...index.clips.slice(1)]});}catch{unsafe=true;}
if(!corrupt||!unsafe||frames!==8892)throw Error('Player acceptance failed');
console.log(JSON.stringify({records:index.clips.length,frames,corruption_rejected:corrupt,path_rejected:unsafe}));
