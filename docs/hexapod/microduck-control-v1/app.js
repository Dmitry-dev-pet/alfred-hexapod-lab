import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {loadModelPackage} from '../model-package.js';
import {DualStickInput} from '../freenove-dual-v8/dual-stick-input.js';
import {forwardKinematics} from './kinematics.js';
import {KinematicPilot} from './control-core.js';
const MODEL_PACKAGE={"schema":"robot-model-package-ref/v1","id":"microduck-original","url":"/assets/robot-models/microduck-original-e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b.json","sha256":"e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b","size":10340979,"source_revision":"bae39f5900b318f7e87fc485c30a31213cfa14eb","mesh_count":38,"instance_count":70,"license_spdx":"CC-BY-NC-SA-4.0"};
const el=id=>document.getElementById(id),stage=el('viewer'),loading=el('pilot-load'),error=el('md-error');
const scene=new THREE.Scene();scene.background=new THREE.Color('#102330');scene.fog=new THREE.Fog('#102330',3,10);
const camera=new THREE.PerspectiveCamera(42,1,.006,30);let renderer=null;
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;stage.appendChild(renderer.domElement);}
catch{el('md-webgl').hidden=false;}
const orbit=new OrbitControls(camera,renderer?.domElement||stage);orbit.enableDamping=true;orbit.minDistance=.10;orbit.maxDistance=5;
scene.add(new THREE.HemisphereLight(0xe2f6ff,0x395166,2.8));
const sun=new THREE.DirectionalLight(0xffffff,2.7);sun.position.set(-1,3,2);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(20,20),new THREE.MeshStandardMaterial({color:0x192e3c,roughness:.94}));floor.rotation.x=-Math.PI/2;floor.position.y=-.003;scene.add(floor);
const grid=new THREE.GridHelper(20,200,0x456f85,0x2b4455);scene.add(grid);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const loader=new STLLoader(),displayed=[],sticks=new DualStickInput(),keys=new Set();
const pads={left:{pad:el('pilot-pad'),thumb:el('pilot-thumb')},right:{pad:el('pilot-pad-right'),thumb:el('pilot-thumb-right')}};
let pilot=null,rig=null,ready=false,previousTime=null,center=new THREE.Vector3();
const digest=async b=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',b)),v=>v.toString(16).padStart(2,'0')).join('');
function resize(){const w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer?.setSize(w,h,false);}
addEventListener('resize',resize);resize();
async function kitBytes(ref){
 if(!/^\/assets\/robot-motions\/microduck-controls-[0-9a-f]{64}\.json$/.test(ref.url)||!Number.isInteger(ref.size)||ref.size>6_000_000)throw Error('Неверная ссылка на движения');
 const cache=globalThis.caches?await caches.open('alfred-microduck-control-motions-v1').catch(()=>null):null;
 let bytes;if(cache){const r=await cache.match(ref.url);if(r){bytes=await r.arrayBuffer();if(bytes.byteLength!==ref.size||await digest(bytes)!==ref.sha256){await cache.delete(ref.url);bytes=null;}}}
 if(!bytes){const r=await fetch(ref.url,{cache:'no-store'});if(!r.ok)throw Error('Движения недоступны: HTTP '+r.status);bytes=await r.arrayBuffer();if(bytes.byteLength!==ref.size||await digest(bytes)!==ref.sha256)throw Error('Не совпала контрольная сумма движений');if(cache)await cache.put(ref.url,new Response(bytes,{headers:{'Content-Type':'application/json'}})).catch(()=>{});}
 return JSON.parse(new TextDecoder().decode(bytes));
}
function drawStick(side){const s=sticks[side];pads[side].thumb.style.left=(50+s.x*38)+'%';pads[side].thumb.style.top=(50-s.y*38)+'%';}
function releaseSticks(){for(const side of ['left','right']){const p=pads[side].pad,id=sticks[side].pointer;if(id!==null&&p.hasPointerCapture(id))p.releasePointerCapture(id);}sticks.reset();keys.clear();drawStick('left');drawStick('right');}
for(const side of ['left','right']){
 const pad=pads[side].pad;
 const point=e=>{const r=pad.getBoundingClientRect(),radius=Math.max(1,Math.min(r.width,r.height)*.38);return{x:(e.clientX-r.x-r.width/2)/radius,y:-(e.clientY-r.y-r.height/2)/radius};};
 pad.addEventListener('pointerdown',e=>{e.preventDefault();e.stopPropagation();if(!ready||pilot.action||pilot.state==='down')return;const p=point(e);if(sticks.start(side,e.pointerId,p.x,p.y)){pad.setPointerCapture(e.pointerId);drawStick(side);}});
 pad.addEventListener('pointermove',e=>{if(sticks[side].pointer!==e.pointerId)return;e.preventDefault();e.stopPropagation();const p=point(e);sticks.move(side,e.pointerId,p.x,p.y);drawStick(side);});
 for(const name of ['pointerup','pointercancel','lostpointercapture'])pad.addEventListener(name,e=>{if(sticks.end(side,e.pointerId)){e.preventDefault();e.stopPropagation();drawStick(side);}});
}
function stop(){releaseSticks();pilot?.stop();}
function fall(){if(!ready)return;if(pilot.paused){pilot.resume();return;}releaseSticks();pilot.fall(el('fall-mode').value);}
function rise(){if(!ready)return;releaseSticks();pilot.rise();}
el('factory-stop').addEventListener('click',stop);el('md-fall').addEventListener('click',fall);el('md-rise').addEventListener('click',rise);
el('reset-pose').addEventListener('click',()=>{releaseSticks();pilot?.reset();cameraReset();});
for(const el of document.querySelectorAll('.md-actions button,.md-actions select'))for(const name of ['pointerdown','pointermove','click'])el.addEventListener(name,e=>e.stopPropagation());
addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA','BUTTON'].includes(document.activeElement?.tagName))return;const k=e.key.toLowerCase();if(['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright',' ','f','g','r'].includes(k)){e.preventDefault();if(k===' ')stop();else if(k==='f')fall();else if(k==='g')rise();else if(k==='r'){releaseSticks();pilot?.reset();cameraReset();}else keys.add(k);}});
addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));addEventListener('blur',stop);
document.addEventListener('visibilitychange',()=>{if(document.hidden){stop();previousTime=null;}});
function applyPose(q){
 const poses=forwardKinematics(rig,q);for(let i=0;i<poses.length;i++){const p=poses[i],m=displayed[i];m.position.set(...p.p);m.quaternion.set(p.q[1],p.q[2],p.q[3],p.q[0]);}
 const now=new THREE.Vector3(q[0],q[2],-q[1]),delta=now.clone().sub(center);camera.position.add(delta);orbit.target.add(delta);center.copy(now);
 stage.dataset.state=pilot.state;stage.dataset.travel=String(Math.hypot(pilot.x,pilot.y).toFixed(4));stage.dataset.head=String(q[rig.servo_qpos.head_yaw].toFixed(4));
}
function cameraReset(){const q=pilot?.pose||[0,0,.12],p=new THREE.Vector3(q[0],q[2],-q[1]);center.copy(p);orbit.target.copy(p);const f=camera.aspect<.9?Math.min(1.65,.9/camera.aspect):1;camera.position.copy(p).add(new THREE.Vector3(.38,.26,.42).multiplyScalar(f));orbit.update();}
el('reset-camera').addEventListener('click',cameraReset);
function ui(){
 const names={standing:'Стойка',walking:'Кинематическая ходьба',falling:'Падение · запись',down:'Лежит · нажмите «Встать»',rising:'Вставание · запись'};
 el('pilot-state').textContent=(pilot.paused?'Пауза · ':'')+names[pilot.state];
 el('pilot-metric').textContent='Положение '+pilot.x.toFixed(2)+' / '+pilot.y.toFixed(2)+' м · курс '+(pilot.heading*180/Math.PI).toFixed(0)+'°';
 el('md-fall').textContent=pilot.paused?'Продолжить':'Упасть';el('md-fall').disabled=!!pilot.action&&!pilot.paused||pilot.state==='down';el('md-rise').disabled=pilot.state!=='down';
 el('fall-mode').disabled=!!pilot.action||pilot.state==='down';
}
async function init(){
 try{
  const [ref,rigData]=await Promise.all([fetch('./motions.json').then(r=>r.json()),fetch('./rig.json').then(r=>r.json())]);
  const [kit,pack]=await Promise.all([kitBytes(ref),loadModelPackage(MODEL_PACKAGE,{onProgress:(n,total)=>loading.textContent='MicroDuck · '+n+' / '+total+' деталей проверено'})]);
  rig=rigData;if(rig.model_package_sha256!==MODEL_PACKAGE.sha256||pack.manifest.visible_geoms.length!==rig.geoms.length)throw Error('Геометрия не совпадает с исходной моделью');
  const meshes=new Map();for(const [name,s] of Object.entries(pack.manifest.meshes)){const g=loader.parse(pack.meshes.get(name));g.scale(...s.scale);const q=s.rotation_wxyz;g.applyQuaternion(new THREE.Quaternion(q[1],q[2],q[3],q[0]));g.translate(...s.translation_xyz);g.computeVertexNormals();meshes.set(name,g);}
  for(let i=0;i<pack.manifest.visible_geoms.length;i++){const s=pack.manifest.visible_geoms[i];if(s.gid!==rig.geoms[i].gid)throw Error('Не совпала деталь исходной модели');const m=new THREE.Mesh(meshes.get(s.mesh),new THREE.MeshStandardMaterial({color:new THREE.Color(...s.rgba.slice(0,3)),transparent:s.rgba[3]<.999,opacity:s.rgba[3],roughness:.62,metalness:.06,side:THREE.DoubleSide}));m.frustumCulled=false;world.add(m);displayed.push(m);}
  pilot=new KinematicPilot(rig,kit);ready=true;applyPose(pilot.pose);cameraReset();ui();loading.hidden=true;el('md-fall').disabled=false;
 }catch(e){error.hidden=false;error.textContent='Не удалось загрузить управление MicroDuck: '+(e.message||e);loading.hidden=true;}
}
function tick(time){
 requestAnimationFrame(tick);const dt=previousTime===null?0:Math.min(.05,(time-previousTime)/1000);previousTime=time;
 if(ready){const dir=(a,b)=>(keys.has(a)?1:0)-(keys.has(b)?1:0),l=sticks.left,r=sticks.right;
  pilot.command(l.pointer!==null?l.y:dir('w','s'),l.pointer!==null?-l.x:dir('a','d'),r.pointer!==null?r.x:dir('arrowright','arrowleft'),r.pointer!==null?r.y:dir('arrowup','arrowdown'));
  applyPose(pilot.tick(dt*Number(el('speed').value||1)));ui();
 }
 orbit.update();renderer?.render(scene,camera);
}
requestAnimationFrame(tick);init();
