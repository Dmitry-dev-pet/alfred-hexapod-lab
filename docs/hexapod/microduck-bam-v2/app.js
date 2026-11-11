import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { loadModelPackage } from '../model-package.js';
import {validateIndex,loadPoseClip} from './pose-loader.js';

// A single immutable package retains all original STL bytes and instance links.
const MODEL_PACKAGE={"schema":"robot-model-package-ref/v1","id":"microduck-original","url":"/assets/robot-models/microduck-original-e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b.json","sha256":"e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b","size":10340979,"source_revision":"bae39f5900b318f7e87fc485c30a31213cfa14eb","mesh_count":38,"instance_count":70,"license_spdx":"CC-BY-NC-SA-4.0"};
const PIN=MODEL_PACKAGE.source_revision;
const MODES={xml_pd:'Позиционные приводы',bam:'BAM M6',bam_backlash:'BAM M6 + люфты'};
const modeSelect=document.getElementById('microduck-mode'),seedSelect=document.getElementById('microduck-seed');
let replayIndex=null;
let indexReady=fetch('./replays.json',{cache:'no-store'}).then(r=>{if(!r.ok)throw Error('Нет каталога 63 записей');return r.json();}).then(d=>replayIndex=validateIndex(d,MODEL_PACKAGE.sha256));
const NAMES={roll180:'Крен 180°',pitch180:'Тангаж 180°',
 side90:'На боку · 90°',fold:'Из сложенного положения',sit:'Из сидячего положения',
 push:'Возмущение скорости',stand:'Контрольная стойка'};
const el=id=>document.getElementById(id);
const stage=el('viewer'),selector=el('microduck-case'),play=el('play'),seek=el('seek'),speed=el('speed');
const loading=el('md-loading'),error=el('md-error'),status=el('md-state'),metric=el('md-metric'),origin=el('md-origin');
const scene=new THREE.Scene();scene.background=new THREE.Color('#0f202d');
scene.fog=new THREE.Fog('#0f202d',2,8);
const camera=new THREE.PerspectiveCamera(42,1,.006,25);
let renderer=null;
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;stage.appendChild(renderer.domElement);}
catch{el('md-webgl').hidden=false;}
const orbit=new OrbitControls(camera,renderer?.domElement||stage);
orbit.enableDamping=true;orbit.dampingFactor=.07;orbit.minDistance=.09;orbit.maxDistance=3.0;
scene.add(new THREE.HemisphereLight(0xe2f6ff,0x395166,2.8));
const sun=new THREE.DirectionalLight(0xffffff,2.7);sun.position.set(-1,3,2);scene.add(sun);
const rim=new THREE.DirectionalLight(0x9fcaff,1.2);rim.position.set(2,2,-1);scene.add(rim);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(9,9),
 new THREE.MeshStandardMaterial({color:0x192e3c,roughness:.94}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.003;scene.add(floor);
const grid=new THREE.GridHelper(6,60,0x456f85,0x2b4455);grid.position.y=.001;scene.add(grid);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world); // MJCF z-up -> Three y-up
const loader=new STLLoader();
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
const previous=new THREE.Vector3();
const displayed=[];
let manifest=null,replay=null,clock=0,playing=false,previousTick=null,hasGeometry=false,requestId=0;

function fail(message){
 error.hidden=false;error.textContent=message;loading.hidden=true;setPlaying(false);
}
function setPlaying(p){playing=!!p;play.textContent=playing?'Ⅱ':'▶';
 play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');}
function resize(){
 const width=Math.max(1,stage.clientWidth),height=Math.max(1,stage.clientHeight);
 camera.aspect=width/height;camera.updateProjectionMatrix();renderer?.setSize(width,height,false);
}
addEventListener('resize',resize);resize();
function validNumberArray(x,n){return Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);}
function validateManifest(m){
 if(m?.schema!=='microduck-original-stl-assets-v1'||m.repo!=='pollen-robotics/microduck_rl'||
   m.revision!==PIN||!m.meshes||!Array.isArray(m.visible_geoms)||
   m.visible_geoms.length<15||m.visible_geoms.length>180)throw Error('Некорректный манифест STL');
 const names=Object.keys(m.meshes);
 if(names.length<8||names.length>80)throw Error('Неверное количество оригинальных STL');
 for(const name of names){
   const a=m.meshes[name];
   if(!/^[A-Za-z0-9_]+$/.test(name)||a.file!==name+'.stl'||
      !/^[0-9a-f]{64}$/.test(a.sha256)||!Number.isInteger(a.size)||
      a.size<84||a.size>8_000_000||
      !validNumberArray(a.scale,3)||!validNumberArray(a.translation_xyz,3)||
      !validNumberArray(a.rotation_wxyz,4)||!Number.isFinite(a.fit_p99_m)||
      a.fit_p99_m>0.00005)throw Error('STL не привязана к оригинальной физической модели: '+name);
 }
 const ids=new Set();
 for(const g of m.visible_geoms){
   if(!Number.isInteger(g.gid)||ids.has(g.gid)||!(g.mesh in m.meshes)||
      !validNumberArray(g.rgba,4))throw Error('Некорректная структура геометрии');
   ids.add(g.gid);
 }
 return m;
}
async function geometryFromBytes(bytes,source){
 const geom=loader.parse(bytes);
 // Convert original STL vertices into the mesh-normalized coordinates that
 // MuJoCo's geom_xpos/geom_xmat refer to; transformation is numerically
 // checked against compiled mesh_vert by the upstream exporter.
 geom.scale(...source.scale);
 const q=source.rotation_wxyz;
 geom.applyQuaternion(new THREE.Quaternion(q[1],q[2],q[3],q[0]));
 geom.translate(...source.translation_xyz);
 geom.computeVertexNormals();
 return geom;
}
async function loadGeometry(){
 loading.textContent='Загружаем единый пакет модели MicroDuck…';
 const pack=await loadModelPackage(MODEL_PACKAGE,{onProgress:(done,total)=>{
   loading.textContent='Единый пакет MicroDuck · '+done+' / '+total+' STL проверено';
 }});
 manifest=validateManifest(pack.manifest);
 const loaded=new Map();
 for(const [name,source] of Object.entries(manifest.meshes)){
   loaded.set(name,await geometryFromBytes(pack.meshes.get(name),source));
 }
 for(const g of manifest.visible_geoms){
   const material=new THREE.MeshStandardMaterial({
     color:new THREE.Color(g.rgba[0],g.rgba[1],g.rgba[2]),
     transparent:g.rgba[3]<.999,opacity:g.rgba[3],
     roughness:.62,metalness:.06,side:THREE.DoubleSide
   });
   const instance=new THREE.Mesh(loaded.get(g.mesh),material);
   instance.frustumCulled=false;world.add(instance);displayed.push(instance);
 }
 hasGeometry=true;
}
function nearest(seconds){
 const frames=replay.frames;let lo=0,hi=frames.length-1;
 while(hi-lo>1){let mid=(hi+lo)>>1;if(frames[mid][0]<seconds)lo=mid;else hi=mid;}
 const a=frames[lo],b=frames[hi],k=Math.max(0,Math.min(1,(seconds-a[0])/(b[0]-a[0]||1)));
 return {a,b,k,lerp:i=>a[i]+(b[i]-a[i])*k};
}
const fmt=t=>Math.floor(t/60)+':'+String(Math.floor(t%60)).padStart(2,'0');
function frame(seconds){
 if(!replay||!hasGeometry)return;
 const {a,b,k,lerp}=nearest(seconds);
 for(let j=0;j<displayed.length;j++){
   const i=8+j*7,m=displayed[j];
   m.position.set(lerp(i),lerp(i+1),lerp(i+2));
   qa.set(a[i+4],a[i+5],a[i+6],a[i+3]);
   qb.set(b[i+4],b[i+5],b[i+6],b[i+3]);
   m.quaternion.copy(qa).slerp(qb,k).normalize();
 }
 const p=new THREE.Vector3(lerp(1),lerp(3),-lerp(2));
 const delta=p.clone().sub(previous);orbit.target.add(delta);camera.position.add(delta);previous.copy(p);
 status.textContent=MODES[replay.mode]+' · '+NAMES[replay.case]+' · старт '+(replay.seed-20261108);
 stage.dataset.clip=[replay.mode,replay.case,replay.seed].join(':');stage.dataset.frame=String(seconds.toFixed(2));
 metric.textContent='t='+seconds.toFixed(2)+' с · высота корпуса '+(lerp(3)*100).toFixed(1)+
   ' см · смещение '+(replay.final_displacement_m*100).toFixed(1)+' см · устойчивость '+(replay.gate_stable_at_end?'PASS':'FAIL');
 origin.textContent='Пик контактов '+replay.peak_floor_resultant_force_n.toFixed(1)+' Н · 300 кадров · '+fmt(seconds)+' / 0:06';
 seek.value=String(seconds);
}
function cameraReset(){
 if(!replay)return;
 const row=replay.frames[0],p=new THREE.Vector3(row[1],row[3],-row[2]);
 const aspect=stage.clientWidth/Math.max(1,stage.clientHeight);
 const f=aspect<.9?Math.min(1.8,.95/aspect):1;
 orbit.target.copy(p);
 camera.position.copy(p).add(new THREE.Vector3(.33,.25,.38).multiplyScalar(f));
 previous.copy(p);orbit.update();
}
play.addEventListener('click',()=>setPlaying(!playing));
seek.addEventListener('input',()=>{clock=Number(seek.value);frame(clock);});
el('reset-camera').addEventListener('click',cameraReset);
async function loadSelected(){
 const nonce=++requestId;error.hidden=true;setPlaying(false);loading.hidden=false;
 loading.textContent='Открываем запись · '+MODES[modeSelect.value]+'…';
 try{
  await indexReady;
  const clip=replayIndex.clips.find(c=>c.mode===modeSelect.value&&c.case===selector.value&&c.seed===Number(seedSelect.value));
  if(!clip)throw Error('Не найдена выбранная запись');
  const payload=await loadPoseClip(clip,replayIndex);if(nonce!==requestId)return;
  replay=payload;clock=0;seek.max='6';cameraReset();frame(0);loading.hidden=true;setPlaying(true);
 }catch(e){if(nonce===requestId)fail('Не удалось открыть физическую запись: '+(e.message||e));}
}
for(const select of [selector,modeSelect,seedSelect])select.addEventListener('change',loadSelected);
async function init(){
 try{await Promise.all([loadGeometry(),indexReady]);await loadSelected();}
 catch(e){
  fail('Не удалось проверить оригинальную модель MicroDuck: '+(e.message||e)+
       '\nПервоначальная загрузка пакета требует интернет; затем проверенная модель сохраняется в браузере. Никакая схематическая модель не подставляется.');
 }
}
let lastTime=null;
function tick(t){
 requestAnimationFrame(tick);
 if(lastTime!==null&&playing&&replay){
   clock=(clock+Math.min(.1,(t-lastTime)/1000)*Number(speed.value||1))%replay.duration_s;
   frame(clock);
 }
 lastTime=t;orbit.update();renderer?.render(scene,camera);
}
requestAnimationFrame(tick);init();
