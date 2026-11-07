import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// Replay only. All body poses are sampled from original MuJoCo FK of measured
// qpos frames. Never simulate physics or generate a gait in the browser.
// Original vendor STL bytes are loaded only onto this user's local device;
// their SHA-256 is checked before decoding. No STL binary in this public repo.
const q = s => document.querySelector(s);
const VENDOR_COMMIT='447945fd40fb5585271dce4ea8cc82567916032a';
const VENDOR_ROOT='https://raw.githubusercontent.com/'+'LuwuDynamics/luwu_mjlab/'+VENDOR_COMMIT+'/';
const ASSET_ROOT='src/assets/robots/lite3/xmls/assets/';
const CACHE_NAME='hexapod-lite3-original-'+VENDOR_COMMIT;
const CACHE_KEY='/hexapod/.private-lite3-mesh-cache/';
const DATA_ROOT='/hexapod/data/';
const CASES={
 'standard-onnx':'lite3-standard-onnx-seed20261106-original-v1.json',
 'standard-zero':'lite3-standard-zero-seed20261106-original-v1.json',
 'midstride-onnx':'lite3-midstride-onnx-seed20261106-original-v1.json',
 'midstride-zero':'lite3-midstride-zero-seed20261106-original-v1.json'
};
const labels={
 'standard-onnx':'ONNX · вперёд / поворот / стоп',
 'standard-zero':'Нулевые действия · 8 с',
 'midstride-onnx':'ONNX · смена направления',
 'midstride-zero':'Нулевые действия · 9,5 с'
};
const stage=q('#viewer'), error=q('#lite3-error'), loading=q('#lite3-status');
const play=q('#play'),seek=q('#seek'),speed=q('#speed'),sel=q('#lite3-case');
const motion=q('#lite3-motion'),contact=q('#lite3-contact'),phase=q('#lite3-phase');
const follow=q('#follow'),showContacts=q('#contacts'),showPath=q('#path');
const scene=new THREE.Scene();scene.background=new THREE.Color('#101e2b');
scene.fog=new THREE.Fog('#101e2b',2.5,9);
const camera=new THREE.PerspectiveCamera(42,1,.01,25);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.4));
renderer.outputColorSpace=THREE.SRGBColorSpace;
stage.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.07;orbit.minDistance=.12;orbit.maxDistance=3;orbit.maxPolarAngle=Math.PI*.5;
scene.add(new THREE.HemisphereLight(0xf1f8ff,0x42546e,2.8));
const sun=new THREE.DirectionalLight(0xffffff,2.3);sun.position.set(-1,4,3);scene.add(sun);
const rim=new THREE.DirectionalLight(0x91c9ed,1.15);rim.position.set(2,1,-2);scene.add(rim);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(15,15),new THREE.MeshStandardMaterial({color:0x152737,roughness:.94}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.003;scene.add(floor);
const grid=new THREE.GridHelper(12,120,0x4b6f84,0x274357);grid.position.y=.001;scene.add(grid);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world); // MJ Z up → Three Y up
const loader=new STLLoader();
const modelMeshes=[];
const loadedMaterial=new THREE.MeshStandardMaterial({color:0x65efb6,metalness:.15,roughness:.52});
const markers=Array.from({length:4},()=>{
 const m=new THREE.Mesh(new THREE.TorusGeometry(.014,.002,6,28),loadedMaterial);
 world.add(m);m.visible=false;return m;
});
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
const aRoot=new THREE.Vector3(),previousRoot=new THREE.Vector3();
let current=null,playing=false,time=0,last=null,trail=null,assetSpec=null,req=0,geometryReady=false;

async function sha256(buf){
 const digest=await crypto.subtle.digest('SHA-256',buf);
 return Array.from(new Uint8Array(digest),b=>b.toString(16).padStart(2,'0')).join('');
}
async function verify(buf,spec){
 if(buf.byteLength!==spec.size)throw Error('Размер STL не совпадает: '+spec.mesh);
 if(await sha256(buf)!==spec.sha256)throw Error('SHA-256 не совпадает: '+spec.mesh);
 return buf;
}
function validateSpec(obj){
 if(obj?.schema!=='xgo-lite3-original-local-stl-assets-v1'||obj.commit!==VENDOR_COMMIT||
    obj.vendor!=='LuwuDynamics/luwu_mjlab'||!Array.isArray(obj.meshes)||obj.meshes.length!==17)
   throw Error('Неверный исходник STL');
 const names=new Set(),bodies=new Set();
 for(const m of obj.meshes){
   if(typeof m.mesh!=='string'||!/^[A-Za-z][A-Za-z0-9_]*\.STL$/.test(m.mesh)||
      !/^[0-9a-f]{64}$/.test(m.sha256)||!Number.isInteger(m.size)||m.size<1000||m.size>5000000||
      typeof m.body!=='string'||!Array.isArray(m.rgba)||m.rgba.length!==4||names.has(m.mesh)||bodies.has(m.body))
     throw Error('Некорректная запись исходной STL');
   names.add(m.mesh);bodies.add(m.body);
 }
 return obj;
}
async function readAsset(spec){
 const filename=spec.mesh;
 // Optionally use a permanently installed private file on disk if available.
 try{
   const local=await fetch('/assets/lite3-original/'+filename,{cache:'no-store'});
   if(local.ok)return await verify(await local.arrayBuffer(),spec);
 }catch(e){if(e.message.startsWith('SHA-256')||e.message.startsWith('Размер STL'))throw e;}
 let cache=null;
 if('caches' in window){try{cache=await caches.open(CACHE_NAME);}catch{}}
 const key=CACHE_KEY+filename;
 if(cache){
   const saved=await cache.match(key);
   if(saved){
     try{return await verify(await saved.arrayBuffer(),spec);}catch{await cache.delete(key);}
   }
 }
 // Only the official public upstream source at the pinned immutable revision.
 // Browser CacheStorage keeps the exact verified bytes private to local origin.
 const url=VENDOR_ROOT+ASSET_ROOT+filename;
 const reply=await fetch(url,{mode:'cors',cache:'no-store'});
 if(!reply.ok)throw Error('Оригинальная STL недоступна: '+filename+' (HTTP '+reply.status+')');
 const payload=await verify(await reply.arrayBuffer(),spec);
 if(cache){
   try{await cache.put(key,new Response(payload.slice(0),{headers:{'Content-Type':'application/octet-stream'}}));}
   catch{console.warn('Lite3 cache quota exceeded; mesh will not persist offline',filename);}
 }
 return payload;
}
async function loadModel(){
 const manifestResponse=await fetch('/hexapod/lite3-original-v1/asset-manifest.json',{cache:'no-store'});
 if(!manifestResponse.ok)throw Error('Нет манифеста оригинальных STL');
 assetSpec=validateSpec(await manifestResponse.json());
 let progress=0;
 let index=0;
 const completed=new Array(assetSpec.meshes.length);
 const worker=async()=>{
  while(index<assetSpec.meshes.length){
   const i=index++,spec=assetSpec.meshes[i];
   const bytes=await readAsset(spec);
   const geom=loader.parse(bytes); // STL vertices are in original MJCF body-local coordinates.
   geom.computeVertexNormals();
   const mtl=new THREE.MeshStandardMaterial({color:new THREE.Color(spec.rgba[0],spec.rgba[1],spec.rgba[2]),metalness:.12,roughness:.5,side:THREE.DoubleSide});
   const mesh=new THREE.Mesh(geom,mtl);
   mesh.frustumCulled=false;
   completed[i]={mesh,body:spec.body};
   loading.textContent='Заводские STL Lite3 · проверено '+(++progress)+' / 17 · локальное хранение';
  }
 };
 await Promise.all([worker(),worker(),worker()]);
 for(const item of completed){world.add(item.mesh);modelMeshes.push(item);}
 geometryReady=true;
}
function validateReplay(obj){
 if(!obj||obj.format!=='xgo-lite3-original-stl-mujoco-replay-v1'||
    obj.source?.vendor_commit!==VENDOR_COMMIT||
    !Array.isArray(obj.frames)||obj.frames.length<60||
    obj.experiment.frame_count!==obj.frames.length||
    obj.layout?.mesh_body_pose_start!==31||obj.layout?.mesh_body_pose_stride!==7||obj.layout?.foot_site_pose_start!==150||obj.layout?.stride!==162||
    obj.layout.body_names.length!==17||
    obj.layout.body_names.some((n,i)=>n!==assetSpec.meshes[i].body))throw Error('Некорректный replay Lite3');
 for(let i=0;i<obj.frames.length;i++){
   const row=obj.frames[i];
   if(row.length!==162||!row.every(Number.isFinite)||(i>0&&row[0]<=obj.frames[i-1][0]))throw Error('Повреждена физическая запись');
 }
 if(Math.abs(obj.frames.at(-1)[0]-obj.experiment.duration_s)>1e-5)throw Error('Не совпадает длительность физики');
 return obj;
}
const getRow=t=>{const frames=current.frames;let lo=0,hi=frames.length-1;
 while(hi-lo>1){const mid=(hi+lo)>>1;if(frames[mid][0]<t)lo=mid;else hi=mid;}
 const a=frames[lo],b=frames[hi],k=Math.max(0,Math.min(1,(t-a[0])/(b[0]-a[0]||1)));
 return {a,b,k,nearest:k<.5?a:b};
};
const interpolate=(a,b,i,k)=>a[i]+(b[i]-a[i])*k;
const fmt=t=>`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;
function resize(){const w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function cameraReset(){if(!current)return;const r=current.frames[0],x=r[31],y=r[32],z=r[33];
 const center=new THREE.Vector3(x,z,-y);orbit.target.copy(center);
 const aspect=stage.clientWidth/Math.max(1,stage.clientHeight),factor=aspect<.9?Math.min(1.75,.95/aspect):1;
 camera.position.copy(center).add(new THREE.Vector3(.28,.20,.32).multiplyScalar(factor));previousRoot.copy(center);orbit.update();
}
function renderFrame(t){if(!current||!geometryReady)return;
 const {a,b,k,nearest}=getRow(t);
 for(let i=0;i<17;i++){
   const mesh=modelMeshes[i].mesh,o=31+i*7;
   mesh.position.set(interpolate(a,b,o,k),interpolate(a,b,o+1,k),interpolate(a,b,o+2,k));
   qa.set(a[o+4],a[o+5],a[o+6],a[o+3]);
   qb.set(b[o+4],b[o+5],b[o+6],b[o+3]);
   mesh.quaternion.copy(qa).slerp(qb,k).normalize();
 }
 const p=new THREE.Vector3(interpolate(a,b,31,k),interpolate(a,b,33,k),-interpolate(a,b,32,k));
 if(follow.checked){const delta=p.clone().sub(previousRoot);camera.position.add(delta);orbit.target.add(delta);}
 previousRoot.copy(p);
 for(let i=0;i<4;i++){
   const force=nearest[22+i],marker=markers[i];
   // Foot site is from genuine MuJoCo FK, force magnitude from saved physics.
   // Ring lies on floor as an XY projection, not a measured contact patch.
   const site=150+3*i;
   marker.position.set(nearest[site],nearest[site+1],.004);
   marker.visible=showContacts.checked&&force>.2;
 }
 phase.textContent=labels[sel.value];
 motion.textContent=`t=${t.toFixed(1)}с · наклон ${nearest[29].toFixed(2)}° · курс vx ${nearest[26].toFixed(2)} / vy ${nearest[27].toFixed(2)} / ω ${nearest[28].toFixed(2)}`;
 contact.textContent=`Контакты (из MuJoCo): ${[0,1,2,3].map((i)=>['FL','FR','RR','RL'][i]+' '+nearest[22+i].toFixed(1)+'Н').join(' · ')}`;
 seek.value=String(t);seek.style.setProperty('--pct',`${100*t/current.experiment.duration_s}%`);q('#clock').textContent=fmt(t);
}
function setPlaying(y){playing=!!y;play.textContent=playing?'Ⅱ':'▶';play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>setPlaying(!playing));
seek.addEventListener('input',()=>{time=Number(seek.value);renderFrame(time);});
showContacts.addEventListener('change',()=>renderFrame(time));
showPath.addEventListener('change',()=>{if(trail)trail.visible=showPath.checked;});
q('#reset-camera').addEventListener('click',cameraReset);
addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT','BUTTON'].includes(document.activeElement?.tagName)){e.preventDefault();setPlaying(!playing);}});
async function loadCase(key){const serial=++req;setPlaying(false);error.hidden=true;loading.textContent='Читаем оригинальные измеренные кадры…';
 try{
   const reply=await fetch(DATA_ROOT+CASES[key],{cache:'no-store'});
   if(!reply.ok)throw Error('Физическая запись отсутствует: '+reply.status);
   const payload=validateReplay(await reply.json());if(serial!==req)return;
   current=payload;time=0;seek.max=String(payload.experiment.duration_s);q('#end-clock').textContent=fmt(payload.experiment.duration_s);
   if(trail){scene.remove(trail);trail.geometry.dispose();trail.material.dispose();trail=null;}
   const pts=payload.frames.map(row=>new THREE.Vector3(row[31],.006,-row[32]));
   trail=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:0x72d6bb,transparent:true,opacity:.78}));scene.add(trail);trail.visible=showPath.checked;
   cameraReset();renderFrame(0);loading.hidden=true;setPlaying(true);
 }catch(ex){if(serial!==req)return;loading.hidden=true;error.hidden=false;error.textContent='Ошибка физической записи Lite3: '+(ex.message||ex);}
}
sel.addEventListener('change',()=>loadCase(sel.value));
async function init(){try{
   if(!crypto?.subtle)throw Error('Для проверки исходных STL требуется браузер с Web Crypto (localhost).');
   await loadModel();
   await loadCase(sel.value);
 }catch(e){loading.hidden=true;error.hidden=false;error.textContent='Оригинальная модель XGO-Lite3 не загружена: '+(e.message||e)+'\nПри первой установке нужен доступ к исходникам LuwuDynamics; проверенные STL затем сохраняются локально. Схематическая модель не подставляется.';setPlaying(false);}}
function tick(ts){requestAnimationFrame(tick);if(last!==null&&playing&&current){const next=time+Math.min(.1,(ts-last)/1000)*Number(speed.value);time=next>=current.experiment.duration_s?next%current.experiment.duration_s:next;renderFrame(time);}last=ts;orbit.update();renderer.render(scene,camera);}
requestAnimationFrame(tick);init();
