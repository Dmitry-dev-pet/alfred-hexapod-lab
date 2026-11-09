// Numeric check using the actual Three.js renderer math and unchanged CAD mesh.
import assert from 'node:assert/strict';
import {readFile,readdir} from 'node:fs/promises';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const dataset=resolve(process.argv[2]),cadPath=resolve(process.argv[3]);
const THREE=await import(pathToFileURL(resolve(process.argv[4])));
const cadBytes=await readFile(cadPath);
assert.equal(createHash('sha256').update(cadBytes).digest('hex'),'8b034155dc50bb6c4e94a55bfa4ebca7b1fb742f2f38fc5c6c9f158c0cae313b');
const cad=JSON.parse(cadBytes);assert.equal(cad.parts.length,73);
let frames=0,transforms=0,maxError=0;
for(const name of (await readdir(dataset+'/replay')).filter(n=>n.endsWith('.json'))){
  const d=JSON.parse(await readFile(dataset+'/replay/'+name));assert.equal(d.body_names.length,9);
  const groups=d.body_names.map(()=>new THREE.Group());
  const points=cad.parts.map(p=>{
    const b=Buffer.from(p.vertices_f32le,'base64');const v=new Float32Array(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));
    const j=d.body_names.indexOf(d.part_body_map[String(p.id)]);assert.ok(j>=0);
    const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(v.slice(0,3)),3));geo.translate(...d.rest_body_positions[j].map(v=>-v));
    return {j,v:[...geo.attributes.position.array]};
  });
  for(const f of d.frames){
    frames++;
    groups.forEach((g,j)=>{const k=1+7*j;g.position.set(f[k],f[k+1],f[k+2]);g.quaternion.set(f[k+4],f[k+5],f[k+6],f[k+3]);g.updateMatrix();});
    for(const {j,v:p} of points){
      const got=new THREE.Vector3(...p).applyMatrix4(groups[j].matrix);const k=1+7*j;
      const [w,x,y,z]=f.slice(k+3,k+7);const dot=x*p[0]+y*p[1]+z*p[2],s=w*w-x*x-y*y-z*z;
      const expected=[2*dot*x+s*p[0]+2*w*(y*p[2]-z*p[1])+f[k],2*dot*y+s*p[1]+2*w*(z*p[0]-x*p[2])+f[k+1],2*dot*z+s*p[2]+2*w*(x*p[1]-y*p[0])+f[k+2]];
      maxError=Math.max(maxError,...got.toArray().map((n,i)=>Math.abs(n-expected[i])));transforms++;
    }
  }
}
assert.equal(frames,2700);assert.equal(transforms,197100);assert.ok(maxError<1e-12);
console.log(JSON.stringify({frames,part_transforms:transforms,max_numeric_residual_m:maxError,actual_webgl_rendered:false}));
