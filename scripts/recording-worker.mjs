// Public immutable recording files. Private sources and write credentials are never used here.
const HASH=/^[a-f0-9]{64}$/;
const MAX=100*1024*1024, CHUNK=8*1024*1024;
export function validManifest(m,sha){
  return m?.schema==='robot-recording-object/v1'&&m.sha256===sha&&HASH.test(sha)&&
    Number.isInteger(m.size)&&m.size>0&&m.size<=MAX&&Array.isArray(m.chunks)&&
    m.chunks.length===Math.ceil(m.size/CHUNK)&&m.chunks.every((c,i)=>
      c.key==='chunk-'+sha+'-'+i&&c.size===Math.min(CHUNK,m.size-i*CHUNK));
}
const headers=m=>({'Content-Type':'application/octet-stream','Content-Length':String(m.size),
  'Cache-Control':'public, max-age=31536000, immutable','ETag':'"'+m.sha256+'"',
  'X-Content-SHA256':m.sha256,'Access-Control-Allow-Origin':'*',
  'Access-Control-Expose-Headers':'ETag, Content-Length, X-Content-SHA256',
  'X-Content-Type-Options':'nosniff'});
export default {async fetch(request,env,ctx){
  const url=new URL(request.url);
  if(url.pathname==='/health')return Response.json({service:'alfred-robot-recordings',readonly:true});
  if(request.method==='OPTIONS')return new Response(null,{status:204,headers:{
    'Access-Control-Allow-Origin':'*','Access-Control-Allow-Methods':'GET, HEAD, OPTIONS'}});
  if(!['GET','HEAD'].includes(request.method))return new Response('Read only',{status:405});
  const match=/^\/records\/([a-f0-9]{64})\.robotreplay$/.exec(url.pathname);
  if(!match)return new Response('Not found',{status:404});
  const sha=match[1],cache=globalThis.caches?.default;
  const key=new Request(url.origin+url.pathname);
  if(request.method==='GET'&&cache){const hit=await cache.match(key);if(hit)return hit;}
  let manifest;
  try{manifest=await env.RECORDINGS.get('manifest-'+sha,'json');}catch{return new Response('Storage unavailable',{status:503});}
  if(!manifest)return new Response('Not found',{status:404});
  if(!validManifest(manifest,sha))return new Response('Invalid object manifest',{status:503});
  if(request.method==='HEAD')return new Response(null,{headers:headers(manifest)});
  let index=0;
  const stream=new ReadableStream({async pull(controller){
    try{if(index===manifest.chunks.length){controller.close();return;}
      const part=manifest.chunks[index++],bytes=await env.RECORDINGS.get(part.key,'arrayBuffer');
      if(!bytes||bytes.byteLength!==part.size)throw Error('Incomplete recording');
      controller.enqueue(new Uint8Array(bytes));
    }catch(error){controller.error(error);}
  }});
  const response=new Response(stream,{headers:headers(manifest)});
  if(cache&&ctx)ctx.waitUntil(cache.put(key,response.clone()).catch(()=>{}));
  return response;
}};
