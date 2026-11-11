import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

// Never create gait or root motion in the browser: each frame is original
// recorded MuJoCo qpos -> mj_forward FK, with raw contact reactions and cmd.
const $=s=>document.querySelector(s);
const DATA=document.documentElement.dataset.replay;
const SOURCE_BLOB='7771e176c42b3aed1d48cc62434771b0fceb7c8c';
const POLICY_SHA='ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712';
const MESH_ROOT='https://raw.githubusercontent.com/G-Levine/pupper_v3_description/1a3c8590189d3a406219d155f4177afc491a7d77/description/meshes/stl/';
const CACHE='alfred-pupper-stanford-meshes-aa475e9a';
const stage=$('#viewer'),play=$('#play'),seek=$('#seek'),status=$('#pupper-status');
const loading=$('#pupper-load'),error=$('#pupper-error');
let record=null,clock=null,time=0,playing=false,lastFollow=null;
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5));
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.setClearColor(0x101923,1);
stage.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x101923);
const camera=new THREE.PerspectiveCamera(48,1,0.01,30);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.08;orbit.minDistance=.16;orbit.maxDistance=5.0;orbit.maxPolarAngle=Math.PI*.53;
scene.add(new THREE.HemisphereLight(0xcce5ff,0x536476,2.35));
const key=new THREE.DirectionalLight(0xffffff,3.1);key.position.set(-1,2,3);scene.add(key);
const fill=new THREE.DirectionalLight(0x88b3d7,1);fill.position.set(2,1,-2);scene.add(fill);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(9,9),new THREE.MeshStandardMaterial({color:0x192b36,roughness:1}));
floor.position.z=-.003;world.add(floor);
const grid=new THREE.GridHelper(8,80,0x38536a,0x293b4a);grid.rotation.x=Math.PI/2;world.add(grid);
const bodyGroups=[],footMarkers=[],visualObjects=[];
const skeletonArray=new Float32Array(16*2*3);
const skeletonGeom=new THREE.BufferGeometry();skeletonGeom.setAttribute('position',new THREE.BufferAttribute(skeletonArray,3));
const skeleton=new THREE.LineSegments(skeletonGeom,new THREE.LineBasicMaterial({color:0x98d8f2,transparent:true,opacity:.9}));world.add(skeleton);
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
const loader=new STLLoader(),loadedByHash=new Map();
const markerGeo=new THREE.TorusGeometry(.013,.0028,6,22);
const markerMat=new THREE.MeshBasicMaterial({color:0x6cf1b0,depthTest:false,transparent:true,opacity:.95});
function resize(){const w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);
 camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function setPlaying(v){playing=!!v;play.textContent=playing?'Ⅱ':'▶';play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>setPlaying(!playing));
seek.addEventListener('input',()=>{time=Number(seek.value);showFrame(time)});
function getRow(t){const rows=record.frames;
 if(t<=rows[0][0])return [rows[0],rows[0],0,0];
 if(t>=rows.at(-1)[0])return [rows.at(-1),rows.at(-1),0,rows.length-1];
 let i=Math.max(0,Math.min(rows.length-2,Math.floor(t*record.frame_rate_hz)-1));
 while(i>0&&rows[i][0]>t)i--;
 while(i<rows.length-2&&rows[i+1][0]<t)i++;
 const a=rows[i],b=rows[i+1];return [a,b,THREE.MathUtils.clamp((t-a[0])/(b[0]-a[0]),0,1),i];}
function getRoot(t){if(!record)return [0,0,.16];const [a,b,f]=getRow(t);
 return [THREE.MathUtils.lerp(a[1],b[1],f),THREE.MathUtils.lerp(a[2],b[2],f),THREE.MathUtils.lerp(a[3],b[3],f)];}
function resetCamera(){const [x,y,z]=getRoot(time);lastFollow=[x,y];
 const center=new THREE.Vector3(x,Math.max(.11,z),-y);orbit.target.copy(center);
 const zoom=camera.aspect<1?Math.min(2.15,1/camera.aspect):1;
 camera.position.copy(center).add(new THREE.Vector3(.29,.24,.33).multiplyScalar(zoom));orbit.update();}
$('#reset-camera').addEventListener('click',resetCamera);
$('#pupper-follow').addEventListener('change',()=>{lastFollow=null;});
resetCamera();
function valid(d){
 if(!DATA||!/^\/hexapod\/data\/pupper-(teleop-20s|jump-9s|turn-fail)-v1\.json$/.test(DATA))throw Error('Unknown replay asset');
 if(d.schema!=='pupper-v3-mujoco-full-commands-replay-v1'||d.robot!=='Stanford Pupper v3'||
    d.control?.policy_sha256!==POLICY_SHA||d.control?.training_performed_by_us!==false||
    d.source?.original_xml_git_blob_sha1!==SOURCE_BLOB||d.source?.visual_source_prefix!==MESH_ROOT||
    d.layout?.stride!==109||d.layout?.body_names?.length!==13||
    d.layout?.foot_site_names?.length!==4||d.layout?.foot_xyz_start!==92||
    d.layout?.foot_fz_start!==104||d.layout?.foot_count_index!==108||
    d.layout?.body_pose_quaternion_order!=='wxyz'||d.frame_rate_hz!==25||
    !Number.isFinite(d.duration_s)||d.duration_s<2||d.duration_s>120||
    d.physics_steps!==Math.round(d.duration_s*250)||
    d.visual_meshes?.length!==13||d.frames?.length!==Math.round(d.duration_s*25)||
    d.cmd_frames?.length!==d.frames.length)throw Error('Invalid Stanford physical replay manifest');
 let previous=0;
 for(let i=0;i<d.frames.length;i++){
   const a=d.frames[i];if(a.length!==109||!a.every(Number.isFinite)||a[0]<=previous||
      a[0]>d.duration_s+1e-5||d.cmd_frames[i].length!==3||!d.cmd_frames[i].every(Number.isFinite))
      throw Error('Corrupted physical frame / command');previous=a[0];
 }
 if(Math.abs(previous-d.duration_s)>1e-4)throw Error('Physical trace truncated');
 const bodies=new Set(d.layout.body_names);
 for(const m of d.visual_meshes){
  if(!bodies.has(m.body)||!/^[A-Za-z0-9_]+\.stl$/.test(m.mesh)||
    !/^[a-f0-9]{40}$/.test(m.git_blob_sha1)||m.size<1000||m.size>2e6||
    m.scale?.length!==3||m.geom_pos?.length!==3||m.geom_quat_wxyz?.length!==4)
   throw Error('Invalid signed mesh metadata');
 }
 return d;
}
function bodyXyz(a,i){const j=1+i*7;return [a[j],a[j+1],a[j+2]];}
function drawSkeleton(a,b,f){let idx=0;const edges=[];
 for(const j of [1,4,7,10])edges.push([0,j]);
 for(const j of [1,4,7,10])edges.push([j,j+1],[j+1,j+2]);
 for(const [j,k] of edges){const p=bodyXyz(a,j),q=bodyXyz(b,j),r=bodyXyz(a,k),s=bodyXyz(b,k);
  for(let n=0;n<3;n++)skeletonArray[idx++]=THREE.MathUtils.lerp(p[n],q[n],f);
  for(let n=0;n<3;n++)skeletonArray[idx++]=THREE.MathUtils.lerp(r[n],s[n],f);
 }
 for(let leg=0;leg<4;leg++){
  const j=3+3*leg,site=92+3*leg,p=bodyXyz(a,j),q=bodyXyz(b,j);
  for(let n=0;n<3;n++)skeletonArray[idx++]=THREE.MathUtils.lerp(p[n],q[n],f);
  for(let n=0;n<3;n++)skeletonArray[idx++]=THREE.MathUtils.lerp(a[site+n],b[site+n],f);
 }
 if(idx!==skeletonArray.length)throw Error('Invalid original FK skeleton');
 skeletonGeom.attributes.position.needsUpdate=true;
}
function activeSegment(time){let seg=record.schedule[0];for(const s of record.schedule){if(s.t<=time+1e-9)seg=s;else break;}return seg;}
function showFrame(time){if(!record)return;
 const [a,b,f,i]=getRow(time);
 for(let n=0;n<bodyGroups.length;n++){const off=1+7*n,body=bodyGroups[n];
  body.position.set(THREE.MathUtils.lerp(a[off],b[off],f),THREE.MathUtils.lerp(a[off+1],b[off+1],f),THREE.MathUtils.lerp(a[off+2],b[off+2],f));
  qa.set(a[off+4],a[off+5],a[off+6],a[off+3]);qb.set(b[off+4],b[off+5],b[off+6],b[off+3]);
  body.quaternion.copy(qa).slerp(qb,f).normalize();
 }
 for(let leg=0;leg<4;leg++){
  const k=92+3*leg,m=footMarkers[leg];m.position.set(THREE.MathUtils.lerp(a[k],b[k],f),THREE.MathUtils.lerp(a[k+1],b[k+1],f),.004);
  m.visible=(f<.5?a:b)[104+leg]>.2;
 }
 drawSkeleton(a,b,f);
 const now=[THREE.MathUtils.lerp(a[1],b[1],f),THREE.MathUtils.lerp(a[2],b[2],f)];
 if($('#pupper-follow').checked){if(lastFollow){const dx=now[0]-lastFollow[0],dy=now[1]-lastFollow[1];
   camera.position.x+=dx;camera.position.z-=dy;orbit.target.x+=dx;orbit.target.z-=dy;}
  lastFollow=now;}
 const loaded=(f<.5?a:b)[108];
 const seg=activeSegment(time);
 const c=record.cmd_frames[Math.min(i,record.cmd_frames.length-1)];
 $('#pupper-distance').textContent=((now[0]-record.frames[0][1])*100).toFixed(1)+' см X';
 $('#pupper-feet').textContent=loaded+'/4';
 $('#pupper-cmd').textContent=`vx ${c[0].toFixed(2)} · vy ${c[1].toFixed(2)} м/с · ω ${c[2].toFixed(2)} рад/с`;
 const jumpEvent=record.jump_events?.some(e=>Math.abs(e.scheduled_t-time)<.28);
 let tag=seg.jump?'команда прыжка':seg.run?'режим бега':jumpEvent?'прыжок':Math.abs(c[2])>.09?'поворот':Math.abs(c[1])>.04?'боковой ход':Math.abs(c[0])>.04?'ходьба':'стойка';
 if(record.status==='failed')tag='ДИАГНОСТИКА · '+tag;
 status.textContent=`${tag} · ${time.toFixed(2)} / ${record.duration_s.toFixed(0)} с · реальная MuJoCo-физика`;
 seek.value=String(time);
}
async function blobHash(buffer){const prefix=new TextEncoder().encode(`blob ${buffer.byteLength}\0`);
 const a=new Uint8Array(prefix.length+buffer.byteLength);a.set(prefix);a.set(new Uint8Array(buffer),prefix.length);
 const digest=await crypto.subtle.digest('SHA-1',a);return [...new Uint8Array(digest)].map(v=>v.toString(16).padStart(2,'0')).join('');}
async function verify(resp,m){if(!resp.ok)throw Error('HTTP '+resp.status);
 const b=await resp.arrayBuffer();if(b.byteLength!==m.size||(await blobHash(b))!==m.git_blob_sha1)throw Error('STL SHA/size mismatch: '+m.mesh);
 return b;}
async function getGeometry(m,cache){if(loadedByHash.has(m.git_blob_sha1))return loadedByHash.get(m.git_blob_sha1);
 const url=MESH_ROOT+m.mesh;let buffer=null;
 try{buffer=await verify(await fetch('/assets/pupper-original/'+m.mesh,{cache:'no-store'}),m)}
 catch(e){if(String(e).includes('mismatch'))throw e;}
 if(!buffer&&cache){const c=await cache.match(url);if(c)try{buffer=await verify(c,m)}catch{await cache.delete(url)}}
 if(!buffer){buffer=await verify(await fetch(url,{mode:'cors',cache:'no-store'}),m);
  if(cache)try{await cache.put(url,new Response(buffer.slice(0),{headers:{'Content-Type':'application/octet-stream'}}))}catch{}}
 const geometry=loader.parse(buffer);geometry.computeVertexNormals();loadedByHash.set(m.git_blob_sha1,geometry);return geometry;}
async function loadMeshes(){if(!crypto?.subtle){loading.textContent='Нет SHA-проверки STL: физический каркас активен';return;}
 let cache=null;try{if('caches' in window)cache=await caches.open(CACHE)}catch{}
 try{
  const index=new Map(record.layout.body_names.map((v,i)=>[v,i]));
  for(let i=0;i<record.visual_meshes.length;i++){
   const m=record.visual_meshes[i];loading.textContent=`Оригинальные STL Stanford: ${i+1}/13`;
   const geom=await getGeometry(m,cache);
   const mesh=new THREE.Mesh(geom,new THREE.MeshStandardMaterial({color:i===0?0x435868:i%3===0?0x8799a6:0xa5adb6,metalness:.12,roughness:.62,side:THREE.DoubleSide}));
   mesh.scale.set(...m.scale);mesh.position.set(...m.geom_pos);const q=m.geom_quat_wxyz;
   mesh.quaternion.set(q[1],q[2],q[3],q[0]).normalize();mesh.frustumCulled=false;
   bodyGroups[index.get(m.body)].add(mesh);visualObjects.push(mesh);
  }
  skeleton.visible=false;loading.textContent='13 оригинальных Stanford STL · хеши проверены';
 }catch(e){skeleton.visible=true;loading.textContent='Оригинальные STL не загрузились — показываем физический каркас: '+(e.message||e)}
}
async function init(){try{
 if(!DATA||!/^\/hexapod\/data\/pupper-(teleop-20s|jump-9s|turn-fail)-v1\.json$/.test(DATA))throw Error('Неверный адрес физической записи');
 const res=await fetch(DATA,{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
 record=valid(await res.json());seek.max=String(record.duration_s);
 const desc=record.title+(record.status==='failed'?' · ОТРИЦАТЕЛЬНЫЙ РЕЗУЛЬТАТ':'');
 $('#pupper-label').textContent=desc;
 for(let i=0;i<record.layout.body_names.length;i++){const b=new THREE.Group();world.add(b);bodyGroups.push(b);}
 for(let i=0;i<4;i++){const m=new THREE.Mesh(markerGeo,markerMat);m.renderOrder=8;world.add(m);footMarkers.push(m);}
 const trail=record.frames.map(r=>new THREE.Vector3(r[1],r[2],.004));
 const path=new THREE.Line(new THREE.BufferGeometry().setFromPoints(trail),new THREE.LineBasicMaterial({color:0x55d4e9,transparent:true,opacity:.85}));world.add(path);
 time=record.frames[0][0];resetCamera();showFrame(time);setPlaying(true);await loadMeshes();
}catch(e){error.hidden=false;error.textContent='Ошибка физической записи: '+(e.message||e);loading.hidden=true;setPlaying(false)}}
function loop(ts){requestAnimationFrame(loop);
 if(clock!==null&&playing&&record){const dt=Math.min(.1,Math.max(0,(ts-clock)/1000))*Number($('#speed').value);
  time+=dt;if(time>record.duration_s){time=time%record.duration_s;resetCamera();}showFrame(time);}
 clock=ts;orbit.update();renderer.render(scene,camera);}
requestAnimationFrame(loop);init();
