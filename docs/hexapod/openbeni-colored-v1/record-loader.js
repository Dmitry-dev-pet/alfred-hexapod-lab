const COMMIT='4545b86a131d05519050df11a424368ef9f06b4f';
let groupRefs={};const packs=new Map();
export function validateIndex(index){
  if(index?.format!=='openbeni-replay-index/v1'||index.source_commit!==COMMIT||index.rig?.body_names?.length!==8||!Array.isArray(index.clips)||index.clips.length!==17)throw Error('Неверный каталог OpenBeni');
  if(index.rig.cad_asset!=='./rig.json'||!/^[a-f0-9]{64}$/.test(index.cad_sha256)||!Number.isSafeInteger(index.cad_size))throw Error('Неверная геометрия');
  for(const clip of index.clips)if(!/^(source|stiff)-[a-z-]+$/.test(clip.id)||clip.url!=='./'+clip.id.split('-')[0]+'-records.json#'+clip.id||!/^[a-f0-9]{64}$/.test(clip.sha256)||!Number.isSafeInteger(clip.size)||clip.size<=0)throw Error('Неверная запись');
  if(new Set(index.clips.map(c=>c.id)).size!==17||!index.clips.some(c=>c.id===index.default_clip))throw Error('Повтор или пропуск записи');
  groupRefs=index.groups;return index;
}
export function decodeRecord(bytes,clip,index){
  const d=JSON.parse(new TextDecoder().decode(bytes));
  if(d.format!=='openbeni-measured-replay/v1'||d.source_commit!==COMMIT||d.trace_sha256!==clip.trace_sha256||d.rig_sha256!==index.cad_sha256||d.stride!==64||d.metric_start!==57||d.visual_count!==8||d.frame_count!==d.frames?.length||d.frames.length<100)throw Error('Неверная трасса');
  let previous=-Infinity;
  for(const row of d.frames){if(row.length!==64||!row.every(Number.isFinite)||row[0]<=previous)throw Error('Повреждены кадры');previous=row[0];for(let i=0;i<8;i++){const k=1+i*7+3;if(Math.abs(Math.hypot(...row.slice(k,k+4))-1)>1e-5)throw Error('Повреждён поворот');}}
  if(Math.abs(d.frames.at(-1)[0]-clip.duration_s)>1e-6)throw Error('Неверная длительность');
  return d;
}
export async function loadVerified(url,size,sha256,decode){
  let bytes;
  if(url.includes('#')){
    const [path,id]=url.split('#');if(!groupRefs[path])throw Error('Неизвестный пакет');
    if(!packs.has(path))packs.set(path,loadVerified(path,groupRefs[path].size,groupRefs[path].sha256,b=>JSON.parse(new TextDecoder().decode(b))));
    const pack=await packs.get(path);if(pack.format!=='openbeni-gzip-record-pack/v1'||typeof pack.records[id]!=='string')throw Error('Нет записи в пакете');
    const compressed=Uint8Array.from(atob(pack.records[id]),v=>v.charCodeAt(0));
    const stream=new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'));
    bytes=new Uint8Array(await new Response(stream).arrayBuffer());
  }else{
    const response=await fetch(url);if(!response.ok)throw Error('HTTP '+response.status);
    bytes=new Uint8Array(await response.arrayBuffer());
  }
  const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),v=>v.toString(16).padStart(2,'0')).join('');
  if(bytes.byteLength!==size||hash!==sha256)throw Error('Контрольная сумма не совпала');
  return decode(bytes);
}
