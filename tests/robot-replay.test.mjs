import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {createHash} from 'node:crypto';
import {gunzipSync,gzipSync} from 'node:zlib';
import {loadReplay,sampleAt} from '../docs/replay/format.js';
const hash=b=>createHash('sha256').update(b).digest('hex');
function block(values,type='f64') { const b=Buffer.alloc(values.length*(type==='f64'?8:4));values.forEach((v,i)=>type==='f64'?b.writeDoubleLE(v,i*8):type==='f32'?b.writeFloatLE(v,i*4):b.writeUInt32LE(v,i*4));return {encoding:type+'le-gzip-base64',count:values.length,sha256:hash(b),data:gzipSync(b).toString('base64')}; }
const encode=d=>gzipSync(Buffer.from(JSON.stringify(d)));
// A different robot, Y-up, with two moving bodies and no contact channels.
const fixture={schema:'robot-replay/v1',metadata:{title:'Two-body format test'},scene:{units:'m',up_axis:'Y',background:'#142632',camera:{follow_node:1,offset:[1,1,1]},lights:[]},nodes:[{id:'a'},{id:'b'}],meshes:[{node:1,vertex_count:3,triangle_count:1,scale:1,translation:[0,0,0],material:{color:'#ffffff',roughness:.5,metalness:0},positions:block([0,0,0,1,0,0,0,1,0],'f32'),indices:block([0,1,2],'u32')}],default_clip:'move',clips:[{id:'move',title:'Move',duration_s:1,frame_count:2,frame_stride:15,frames:block([0,0,0,0,1,0,0,0,1,0,0,1,0,0,0,1,1,0,0,1,0,0,0,2,0,0,1,0,0,0]),environment:[]}]};
const model=await loadReplay(encode(fixture));assert.equal(model.doc.nodes.length,2);assert.equal(model.clips[0].contacts,null);assert.deepEqual(sampleAt(model.clips[0],.5),{a:0,b:1,k:.5});assert.deepEqual(sampleAt(model.clips[0],10),{a:1,b:1,k:0});
for(const change of [d=>d.schema='robot-replay/v2',d=>d.meshes[0].positions.sha256='0'.repeat(64),d=>d.nodes[1].id='a',d=>d.meshes[0].indices=block([0,1,3],'u32'),d=>d.clips[0].frames=block([0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,1,1,0,0,1,0,0,0,2,0,0,1,0,0,0]),d=>d.clips[0].frame_stride=14]){const bad=structuredClone(fixture);change(bad);await assert.rejects(loadReplay(encode(bad)));}
let sourceVerified=false;
if(process.argv[2]) {
 const input=await fs.readFile(process.argv[2]), loaded=await loadReplay(input);assert.equal(loaded.clips.reduce((n,c)=>n+c.frame_count,0),3400);assert.equal(loaded.meshes.length,19);
 const source=process.argv[3];if(source){const rig=JSON.parse(await fs.readFile(path.join(source,'rig.json')));const index=JSON.parse(await fs.readFile(path.join(source,'replays.json')));
 for(const [i,m] of loaded.meshes.entries()){const p=rig.parts[i];const vp=JSON.parse(await fs.readFile(path.join(source,p.vertices_f32le_gzip_resource.path))),ip=JSON.parse(await fs.readFile(path.join(source,p.indices_u32le_gzip_resource.path)));assert.deepEqual(Buffer.from(m.positions.buffer),gunzipSync(Buffer.from(vp.data,'base64')));assert.deepEqual(Buffer.from(m.indices.buffer),gunzipSync(Buffer.from(ip.data,'base64')));assert.deepEqual(m.translation,p.local_translation_m);}
 for(const c of loaded.clips){const meta=index.clips.find(x=>x.id===c.id),record=JSON.parse(await fs.readFile(path.join(source,meta.record.path)));for(let i=0;i<c.frame_count;i++){const original=record.replay.frames[i];assert.equal(c.frames[i*c.frame_stride],original[0]);for(let j=0;j<c.frame_stride-1;j++)assert.equal(c.frames[i*c.frame_stride+1+j],original[8+j]);assert.deepEqual([...c.contacts.subarray(i*6,(i+1)*6)],record.foot_normal_load_n[i]);}assert.equal(c.source_trace_sha256,meta.source_trace_sha256);}
 sourceVerified=true;
 }
 console.log(JSON.stringify({sourceVerified,clips:loaded.clips.map(c=>c.id),frames:3400,meshes:19,triangles:loaded.meshes.reduce((n,m)=>n+m.triangle_count,0),packageBytes:input.length}));
}
console.log('PASS: alternate robot, interpolation, invalid version, digest, indices, nodes, quaternion and stride rejection'+(sourceVerified?', exact source mesh/pose/contact identity':''));
