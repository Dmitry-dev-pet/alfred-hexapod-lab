import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// Replay ONLY. The trained Stanford neural policy (20x48 obs) generated actions.
// Physical states were generated with 1250 genuine MuJoCo mj_step calls.
// Browser does no dynamics, gait control, inverse kinematics or RL inference.
// Displayed body transforms and feet are mj_forward of recorded qpos; only
// visual interpolation BETWEEN recorded 50Hz frames is done here.
const $ = s => document.querySelector(s);
const DATA = '/hexapod/data/pupper-v3-cpu-pretrained-walk-5s-v1.json';
const COMMIT = 'aa475e9a393a209b29fd5de7e0ee10f3de643230';
const ROOT = "https://raw.githubusercontent.com/G-Levine/pupper_v3_description/1a3c8590189d3a406219d155f4177afc491a7d77/description/meshes/stl/";
const SOURCE_BLOB = '7771e176c42b3aed1d48cc62434771b0fceb7c8c';
const CACHE_NAME = 'alfred-pupper-stanford-meshes-'+COMMIT;
const stage = $('#viewer'), seek = $('#seek'), play = $('#play');
const status = $('#pupper-status'), loading = $('#pupper-load'), error = $('#pupper-error');
let dataset=null, t=0, playing=false, lastClock=null, meshesLoaded=false, lastFollowX=null;
const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.setClearColor(0x101923,1);
stage.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x101923);
const camera=new THREE.PerspectiveCamera(47,1,0.01,15);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=0.08;
orbit.minDistance=.16;orbit.maxDistance=2.2;orbit.maxPolarAngle=Math.PI*.52;
scene.add(new THREE.HemisphereLight(0xc8e8ff,0x536476,2.4));
const light=new THREE.DirectionalLight(0xffffff,3.2);light.position.set(-1,2,3);scene.add(light);
const fill=new THREE.DirectionalLight(0x88b3d7,1.0);fill.position.set(2,1,-2);scene.add(fill);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(4,4),new THREE.MeshStandardMaterial({color:0x192b36,roughness:1}));
floor.position.z=-.002;world.add(floor);
const grid=new THREE.GridHelper(2,40,0x38536a,0x293b4a);grid.rotation.x=Math.PI/2;
world.add(grid);
const loader=new STLLoader();
const bodyGroups=[];
const footMarkers=[];
const markerGeo=new THREE.TorusGeometry(.013,.0028,6,22);
const markerMaterial=new THREE.MeshBasicMaterial({color:0x66deb4,depthTest:false,transparent:true,opacity:.94});
const lineCount=16;
const skeletonPts=new Float32Array(lineCount*2*3);
const skeletonGeom=new THREE.BufferGeometry();skeletonGeom.setAttribute('position',new THREE.BufferAttribute(skeletonPts,3));
const skeleton=new THREE.LineSegments(skeletonGeom,new THREE.LineBasicMaterial({color:0x96d1eb,transparent:true,opacity:.9}));
world.add(skeleton);
const visualObjects=[];
let actualBodyTrail=null;
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();

function resize(){
 const w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);
 camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);
}
window.addEventListener('resize',resize);
function resetCamera(){
 const x=dataset?dataset.frames[Math.max(0,Math.min(dataset.frames.length-1,Math.floor(t*50)-1))][1]:0;
 const center=new THREE.Vector3(x,.115,0);
 lastFollowX=x;
 orbit.target.copy(center);
 const zoom=camera.aspect<1?Math.min(2.15,1/camera.aspect):1;
 camera.position.copy(center).add(new THREE.Vector3(.27,.19,.3).multiplyScalar(zoom));
 orbit.update();
}
resize();resetCamera();
$('#reset-camera').addEventListener('click',resetCamera);

function validate(d){
 if(d.schema!=='pupper-v3-mujoco-cpu-pretrained-walk-replay-v1'||d.is_walk!==true||
    d.control?.policy_sha256!=='ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712'||
    d.control?.in_shape?.[1]!==960||d.control?.training_performed_by_us!==false||
    d.control?.servo_delay_physics_steps!==4||d.source?.commit!==COMMIT||
    d.source?.original_xml_git_blob_sha1!==SOURCE_BLOB||d.source?.visual_source_prefix!==ROOT||
    d.physics_steps!==1250||d.duration_s!==5||d.frame_rate_hz!==50||
    d.layout?.stride!==109||d.layout?.body_names?.length!==13||d.layout?.foot_site_names?.length!==4||
    d.layout?.body_pose_start!==1||d.layout?.foot_xyz_start!==92||d.layout?.foot_fz_start!==104||
    d.layout?.foot_count_index!==108||d.layout?.body_pose_quaternion_order!=='wxyz'||
    d.visual_meshes?.length!==13||!Array.isArray(d.frames)||d.frames.length!==250){
   throw Error('Несовместимый или неполный физический архив Stanford');
 }
 let lastTime=0;
 for(const row of d.frames){
   if(row.length!==109||!row.every(Number.isFinite)||row[0]<=lastTime||row[0]>5.000001)
     throw Error('Повреждённый физический кадр');
   lastTime=row[0];
 }
 if(Math.abs(lastTime-5)>1e-5)throw Error('Некорректная длительность записи');
 const names=new Set(d.layout.body_names);
 for(const m of d.visual_meshes){
  if(!names.has(m.body)||!/^[A-Za-z0-9_]+\.stl$/.test(m.mesh)||!Number.isInteger(m.size)||
    m.size<1000||m.size>2e6||!/^[a-f0-9]{40}$/.test(m.git_blob_sha1)||
    m.scale?.length!==3||m.geom_pos?.length!==3||m.geom_quat_wxyz?.length!==4){
    throw Error('Неверный оригинальный STL-манифест');
  }
 }
 return d;
}

function setPlaying(value){
 playing=!!value;
 play.textContent=playing?'Ⅱ':'▶';
 play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');
}
play.addEventListener('click',()=>setPlaying(!playing));
seek.addEventListener('input',()=>{t=Number(seek.value);showFrame(t)});
$('#pupper-follow').addEventListener('change',()=>{lastFollowX=null;});

function sample(t){
 const rows=dataset.frames;
 if(t<=rows[0][0])return [rows[0],rows[0],0];
 if(t>=rows[rows.length-1][0])return [rows.at(-1),rows.at(-1),0];
 let i=Math.min(rows.length-2,Math.max(0,Math.floor(t/.02)-1));
 while(i>0&&rows[i][0]>t)i--;
 while(i<rows.length-2&&rows[i+1][0]<t)i++;
 const a=rows[i],b=rows[i+1],fraction=(t-a[0])/(b[0]-a[0]);
 return [a,b,THREE.MathUtils.clamp(fraction,0,1)];
}
function xyz(row,bodyIndex){const j=1+bodyIndex*7;return [row[j],row[j+1],row[j+2]]}
function fzValue(t,v0,v1,k){return t<.5?v0[104+k]:v1[104+k]}
function moveSkeleton(a,b,f){
 const edges=[];
 for(const front of [1,4,7,10])edges.push([0,front]);
 for(const front of [1,4,7,10])edges.push([front,front+1],[front+1,front+2]);
 let write=0;
 for(const [j,k] of edges){
  const p=xyz(a,j),q=xyz(b,j),r=xyz(a,k),s=xyz(b,k);
  for(let u=0;u<3;u++)skeletonPts[write++]=THREE.MathUtils.lerp(p[u],q[u],f);
  for(let u=0;u<3;u++)skeletonPts[write++]=THREE.MathUtils.lerp(r[u],s[u],f);
 }
 for(let leg=0;leg<4;leg++){
  const base=3+leg*3,root=92+leg*3;
  const p=xyz(a,base),q=xyz(b,base);
  for(let u=0;u<3;u++)skeletonPts[write++]=THREE.MathUtils.lerp(p[u],q[u],f);
  for(let u=0;u<3;u++)skeletonPts[write++]=THREE.MathUtils.lerp(a[root+u],b[root+u],f);
 }
 if(write!==skeletonPts.length)throw Error('Неправильный кинематический каркас');
 skeletonGeom.attributes.position.needsUpdate=true;
}
function showFrame(time){
 if(!dataset)return;
 const [a,b,f]=sample(time);
 for(let i=0;i<bodyGroups.length;i++){
  const base=1+7*i,body=bodyGroups[i];
  body.position.set(
   THREE.MathUtils.lerp(a[base],b[base],f),
   THREE.MathUtils.lerp(a[base+1],b[base+1],f),
   THREE.MathUtils.lerp(a[base+2],b[base+2],f));
  qa.set(a[base+4],a[base+5],a[base+6],a[base+3]);
  qb.set(b[base+4],b[base+5],b[base+6],b[base+3]);
  body.quaternion.copy(qa).slerp(qb,f).normalize();
 }
 for(let i=0;i<4;i++){
  const off=92+i*3,marker=footMarkers[i];
  marker.position.set(
   THREE.MathUtils.lerp(a[off],b[off],f),
   THREE.MathUtils.lerp(a[off+1],b[off+1],f),
   .003);
  const force=f<.5?a[104+i]:b[104+i];
  marker.visible=force>.2;
 }
 moveSkeleton(a,b,f);
 const root=1;
 const height=THREE.MathUtils.lerp(a[root+2],b[root+2],f);
 const distance=THREE.MathUtils.lerp(a[1],b[1],f)-dataset.frames[0][1];
 $('#pupper-distance').textContent=(distance*100).toFixed(1)+' см';
 if($('#pupper-follow').checked){if(lastFollowX!==null){const dx=THREE.MathUtils.lerp(a[1],b[1],f)-lastFollowX;camera.position.x+=dx;orbit.target.x+=dx;}lastFollowX=THREE.MathUtils.lerp(a[1],b[1],f);}
 const loaded=(f<.5?a:b)[108];
 $('#pupper-feet').textContent=loaded+'/4';
 seek.value=String(time);
 status.textContent=`Готовая сеть Stanford · ${time.toFixed(2)} / 5 с · физика MuJoCo · ${loaded} опоры`;
}

async function gitBlobDigest(bytes){
 const prefix=new TextEncoder().encode(`blob ${bytes.byteLength}\0`);
 const p=new Uint8Array(prefix.byteLength+bytes.byteLength);
 p.set(prefix);p.set(new Uint8Array(bytes),prefix.byteLength);
 const dig=await crypto.subtle.digest('SHA-1',p);
 return [...new Uint8Array(dig)].map(v=>v.toString(16).padStart(2,'0')).join('');
}
async function checkedBytes(response,m){
 if(!response.ok)throw Error('HTTP '+response.status+' '+m.mesh);
 const bytes=await response.arrayBuffer();
 if(bytes.byteLength!==m.size||(await gitBlobDigest(bytes))!==m.git_blob_sha1)
   throw Error('Контрольная сумма оригинальной STL не совпала: '+m.mesh);
 return bytes;
}
const geometryByBlob=new Map();
async function getGeometry(m,cache){
 if(geometryByBlob.has(m.git_blob_sha1))return geometryByBlob.get(m.git_blob_sha1);
 const url=ROOT+m.mesh;
 const local='/assets/pupper-original/'+m.mesh;
 let buffer=null;
 try{buffer=await checkedBytes(await fetch(local,{cache:'no-store'}),m)}catch(e){if(e.message.startsWith('Контрольная'))throw e;}
 if(!buffer&&cache){
  const cached=await cache.match(url);
  if(cached)try{buffer=await checkedBytes(cached,m)}catch{await cache.delete(url)}
 }
 if(!buffer){
  buffer=await checkedBytes(await fetch(url,{mode:'cors',cache:'no-store'}),m);
  if(cache)try{await cache.put(url,new Response(buffer.slice(0),{headers:{'Content-Type':'application/octet-stream'}}))}catch{}
 }
 const geometry=loader.parse(buffer);
 geometry.computeVertexNormals();
 geometryByBlob.set(m.git_blob_sha1,geometry);
 return geometry;
}
async function loadVisualSTLs(){
 if(!crypto?.subtle){loading.textContent='Нет криптографической проверки STL. Показан физический каркас.';return;}
 let cache=null;
 try{if('caches' in window)cache=await caches.open(CACHE_NAME)}catch{}
 try{
  const idx=new Map(dataset.layout.body_names.map((name,i)=>[name,i]));
  for(let i=0;i<dataset.visual_meshes.length;i++){
   const m=dataset.visual_meshes[i];
   loading.textContent=`Исходные STL Stanford: ${i+1}/13 · проверка Git SHA-1…`;
   const geometry=await getGeometry(m,cache);
   const color=i===0?0x435868:i%3===0?0x8799a6:0xa5adb6;
   const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,roughness:.62,metalness:.12,side:THREE.DoubleSide}));
   mesh.scale.set(...m.scale);
   mesh.position.set(...m.geom_pos);
   const q=m.geom_quat_wxyz;mesh.quaternion.set(q[1],q[2],q[3],q[0]).normalize();
   mesh.frustumCulled=false;
   bodyGroups[idx.get(m.body)].add(mesh);
   visualObjects.push(mesh);
  }
  meshesLoaded=true;
  skeleton.visible=false;
  loading.textContent='13 оригинальных STL Stanford · локальный кэш · источник закреплён по SHA';
 }catch(e){
  skeleton.visible=true;
  loading.textContent='STL недоступны: показан каркас по записанной кинематике. '+(e.message||e);
 }
}
async function init(){
 try{
  const res=await fetch(DATA,{cache:'no-store'});
  if(!res.ok)throw Error('Физическая запись недоступна: HTTP '+res.status);
  dataset=validate(await res.json());
  for(let i=0;i<dataset.layout.body_names.length;i++){const grp=new THREE.Group();world.add(grp);bodyGroups.push(grp)}
  for(let i=0;i<4;i++){
   const m=new THREE.Mesh(markerGeo,markerMaterial);m.renderOrder=8;world.add(m);footMarkers.push(m);
  }
  // Actual trajectory of the measured base x/y, not a fabricated gait.
  const points=dataset.frames.map(r=>new THREE.Vector3(r[1],r[2],.003));
  actualBodyTrail=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({color:0x66d8ed,transparent:true,opacity:.87}));
  world.add(actualBodyTrail);
  t=dataset.frames[0][0];resetCamera();showFrame(t);setPlaying(true);
  // STL is cosmetic and optional; physics data is available without internet.
  await loadVisualSTLs();
 }catch(e){error.hidden=false;error.textContent='Не удалось загрузить физическую запись: '+(e.message||e);loading.hidden=true;setPlaying(false);}
}
function loop(ts){
 requestAnimationFrame(loop);
 if(lastClock!==null&&playing&&dataset){
  const dt=Math.min(.10,Math.max(0,(ts-lastClock)/1000))*Number($('#speed').value);
  t+=dt;
  if(t>5){t=t%5;resetCamera();}
  showFrame(t);
 }
 lastClock=ts;
 orbit.update();renderer.render(scene,camera);
}
requestAnimationFrame(loop);
init();
