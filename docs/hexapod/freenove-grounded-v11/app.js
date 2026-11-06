import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {GAITS,validateFactoryKit} from '../freenove-original-v7/factory-controller.js';
import {walkingInput,rightInput} from '../freenove-combined-v9/combo-controller.js';
import {PlantedCombinedPilot} from './planted-controller.js';
import {GroundLock} from './ground-lock.js';
import {DualStickInput} from '../freenove-dual-v8/dual-stick-input.js';

// Freenove OEM tables + offline planted yaw and contact-anchored planar preview.
// Reference FK + least-squares planar alignment only; no runtime MuJoCo physics.
const viewer=document.getElementById('viewer');
const status=document.getElementById('pilot-load');
const stats=document.getElementById('pilot-state');
const readout=document.getElementById('pilot-speed-readout');
const odom=document.getElementById('pilot-odom');
const pads={
 left:{pad:document.getElementById('pilot-pad'),thumb:document.getElementById('pilot-thumb')},
 right:{pad:document.getElementById('pilot-pad-right'),thumb:document.getElementById('pilot-thumb-right')}
};
const sticks=new DualStickInput();
const steerSelect=document.getElementById('steer-mode'),rightSelect=document.getElementById('right-mode'),gaitSelect=document.getElementById('factory-gait');
const tempo=document.getElementById('factory-tempo'),rate=document.getElementById('factory-rate');
const poseSliders=[...['height','yaw'].map(key=>({key,el:document.getElementById('pose-'+key),out:document.getElementById('pose-'+key+'-value')}))];
const scene=new THREE.Scene();scene.background=new THREE.Color('#101b24');
const camera=new THREE.PerspectiveCamera(43,1,.01,30);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.075;orbit.minDistance=.26;orbit.maxDistance=5;orbit.maxPolarAngle=Math.PI*.49;
scene.add(new THREE.HemisphereLight(0xd8ebf7,0x435561,2.6));
const sun=new THREE.DirectionalLight(0xffffff,2.3);sun.position.set(-1.5,3.5,2.6);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(50,50),new THREE.MeshStandardMaterial({color:0x15242e,roughness:1}));floor.rotation.x=-Math.PI/2;floor.position.y=-.004;scene.add(floor);
const grid=new THREE.GridHelper(8,80,0x5d8893,0x28414c);grid.position.y=.002;scene.add(grid);
const physics=new THREE.Group();physics.rotation.x=-Math.PI/2;scene.add(physics);
const cadRoot=new THREE.Group();physics.add(cadRoot);
const vec=a=>new THREE.Vector3(a[0],a[1],a[2]);
const quat=q=>new THREE.Quaternion(q[1],q[2],q[3],q[0]);
const zaxis=new THREE.Vector3(0,0,1);
let ref=null,factory=null,headKit=null,plantedKit=null,pilot=null,groundLock=null,groundInfo=null,ready=false,pose=null,prev=null,seconds=0;
let bodyNodes=new Map(),cameraCenter=new THREE.Vector3();
const keys=new Set();
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function groundCenter(){
 if(!factory)return new THREE.Vector3();
 const p=vec(factory.cad_adapter.neutral_root_xyz_m).add(vec(pose?.rootOffsetWorld||[0,0,0]));
 const ground=groundLock?.pose||{x:0,y:0,yaw:0};
 p.applyAxisAngle(zaxis,ground.yaw);
 p.x+=ground.x;p.y+=ground.y;
 return new THREE.Vector3(p.x,p.z,-p.y);
}
function cameraReset(){
 if(!factory||!pilot)return;
 const c=groundCenter();
 const factor=Math.min(2,Math.max(1,1/Math.max(.52,viewer.clientWidth/Math.max(1,viewer.clientHeight))));
 orbit.target.copy(c);camera.position.copy(c).add(new THREE.Vector3(.54,.40,.68).multiplyScalar(factor));cameraCenter.copy(c);orbit.update();
}
document.getElementById('reset-camera').addEventListener('click',e=>{e.stopPropagation();cameraReset();});
function material(rgba){return new THREE.MeshStandardMaterial({color:new THREE.Color(...rgba.slice(0,3)),metalness:.22,roughness:.56,side:THREE.DoubleSide,transparent:rgba[3]<.995,opacity:rgba[3],depthWrite:rgba[3]>=.995});}
function proxy(g){if(g.kind==='cylinder'){
 const geom=new THREE.CylinderGeometry(g.size[0],g.size[0],2*g.size[1],16);geom.rotateX(Math.PI/2);return geom;}
 return new THREE.BoxGeometry(...g.size.map(v=>2*v));}
async function loadCAD(){
 const response=await fetch('../data/cad-rig-live-v1-slow.json',{cache:'no-store'});
 if(!response.ok)throw Error('Нет реальной CAD30-метаинформации: '+response.status);
 const rig=await response.json();
 if(rig.format!=='fnk0052-mujoco-cad-visual-replay-v1'||rig.meshes.length!==30||rig.bodies.length!==27)throw Error('CAD30 не совпадает с проверенным набором');
 for(const b of rig.bodies){const n=new THREE.Group();cadRoot.add(n);n.name=b.name;bodyNodes.set(b.id,n);}
 const loader=new STLLoader(),byId=new Map();let next=0;const errors=[];let downloaded=0;
 async function worker(){while(next<rig.meshes.length){const mesh=rig.meshes[next++];
  try{const geometry=await loader.loadAsync(mesh.url);geometry.translate(...mesh.mesh_pos.map(x=>-x));geometry.applyQuaternion(quat(mesh.mesh_quat_wxyz).invert());byId.set(mesh.id,geometry);downloaded++;status.textContent='CAD30 · загружено '+downloaded+'/30';}
  catch(e){errors.push(mesh.filename+': '+e.message);}}}
 await Promise.all(Array.from({length:5},()=>worker()));
 if(downloaded<30)throw Error(`Нет локальных STL (${downloaded}/30) — ${errors.slice(0,2).join('; ')}`);
 const mats=new Map();
 for(const g of rig.geoms){const parent=bodyNodes.get(g.body);if(!parent)throw Error('Неизвестный CAD body '+g.body);
  const key=g.rgba.join(',');if(!mats.has(key))mats.set(key,material(g.rgba));
  const shape=g.kind==='mesh'?byId.get(g.mesh):proxy(g);if(!shape)continue;
  const n=new THREE.Group();n.position.copy(vec(g.pos));n.quaternion.copy(quat(g.quat_wxyz));n.add(new THREE.Mesh(shape,mats.get(key)));parent.add(n);
 }
}
// Reference-FK in the original CAD axes; planted-foot tracking is algebra only.
function applyPose(s){
 const root=vec(factory.cad_adapter.neutral_root_xyz_m).add(vec(s.rootOffsetWorld));
 const r=vec(s.bodyRotvecWorld),a=r.length();
 const attitude=a>1e-9?new THREE.Quaternion().setFromAxisAngle(r.clone().divideScalar(a),a):new THREE.Quaternion();
 const baseRotation=attitude.multiply(quat(ref.root_quat_wxyz));
 const frames=new Map();frames.set(1,{p:root,q:baseRotation});
 for(const node of ref.skeleton){
  const parent=frames.get(node.parent);if(!parent)throw Error('Нарушен граф тела '+node.parent);
  const p=vec(node.p),q=quat(node.q);
  if(node.adr!==undefined){
   const anchor=vec(node.anchor),axis=vec(node.axis),delta=s.q[node.adr]-ref.q_stand[node.adr];
   const dq=new THREE.Quaternion().setFromAxisAngle(axis,delta);
   p.sub(anchor).applyQuaternion(dq).add(anchor);q.premultiply(dq);
  }
  p.applyQuaternion(parent.q).add(parent.p);q.premultiply(parent.q);frames.set(node.id,{p,q});
 }
 const toes={};
 for(const [name,info] of Object.entries(plantedKit.cad_tip_local)){
  const f=frames.get(info.body);if(!f)throw Error('Missing CAD toe body '+name);
  const point=vec(info.tip_local_xyz_m).applyQuaternion(f.q).add(f.p);
  toes[name]={x:point.x,y:point.y,z:point.z};
 }
 groundInfo=groundLock.step(toes);
 const {x,y,yaw}=groundInfo.pose;
 const turn=new THREE.Quaternion().setFromAxisAngle(zaxis,yaw),move=new THREE.Vector3(x,y,0);
 for(const [id,obj] of bodyNodes){const f=frames.get(id);if(f){obj.position.copy(f.p).applyQuaternion(turn).add(move);obj.quaternion.copy(turn).multiply(f.q);}}
 const c=groundCenter(),delta=c.clone().sub(cameraCenter);
 camera.position.add(delta);orbit.target.add(delta);cameraCenter.copy(c);
}
// One active pointer per side. Each pad captures only its own touch; releasing one
// finger leaves the other command active. Camera gestures outside both pads survive.
function padCoords(pad,e){
 const r=pad.getBoundingClientRect(),rad=Math.max(1,Math.min(r.width,r.height)*.38);
 return {x:(e.clientX-r.left-r.width/2)/rad,y:-(e.clientY-r.top-r.height/2)/rad};
}
function drawStick(side){
 const {thumb}=pads[side];const s=sticks[side];
 const x=s.x,y=s.y;
 thumb.style.left=`${50+x*38}%`;thumb.style.top=`${50-y*38}%`;
}
for(const side of ['left','right']){
 const {pad}=pads[side];
 pad.addEventListener('pointerdown',e=>{
  e.preventDefault();e.stopPropagation();
  const p=padCoords(pad,e);
  if(!sticks.start(side,e.pointerId,p.x,p.y))return;
  pad.setPointerCapture(e.pointerId);drawStick(side);
 });
 pad.addEventListener('pointermove',e=>{
  if(sticks[side].pointer!==e.pointerId)return;
  e.preventDefault();e.stopPropagation();
  const p=padCoords(pad,e);sticks.move(side,e.pointerId,p.x,p.y);drawStick(side);
 });
 for(const type of ['pointerup','pointercancel','lostpointercapture']){
  pad.addEventListener(type,e=>{
   if(sticks.end(side,e.pointerId)){e.preventDefault();e.stopPropagation();drawStick(side);}
  });
 }
}
function resetSliders(){
 for(const {key,el,out} of poseSliders){el.value='0';out.textContent='0'+(key==='height'?' мм':'°');}
}
function clearPad(side){
 const pad=pads[side].pad,ptr=sticks[side].pointer;
 if(ptr!==null){try{if(pad.hasPointerCapture(ptr))pad.releasePointerCapture(ptr);}catch{}}
 sticks.clear(side);drawStick(side);
}
function neutralize(){
 sticks.reset();for(const side of ['left','right'])drawStick(side);keys.clear();resetSliders();pilot?.stop();
}
document.getElementById('factory-stop').addEventListener('click',e=>{e.stopPropagation();neutralize();});
document.getElementById('reset-pose').addEventListener('click',e=>{
 e.stopPropagation();clearPad('right');resetSliders();pilot?.resetPose();
});
function rightLabel(){
 const m=rightSelect.value;
 const labels={tilt:'КОРПУС · НАКЛОН',shift:'КОРПУС · СДВИГ',head:'ГОЛОВА · ПОВОРОТ'};
 const aria={tilt:'Правый джойстик: крен и тангаж корпуса',shift:'Правый джойстик: смещение корпуса вперед-назад и вбок',head:'Правый джойстик: поворот и наклон головы'};
 document.getElementById('right-pad-label').textContent=labels[m];pads.right.pad.setAttribute('aria-label',aria[m]);
}
rightSelect.addEventListener('change',()=>{clearPad('right');rightLabel();});
steerSelect.addEventListener('change',()=>{
 document.getElementById('pad-label').textContent=steerSelect.value==='turn'?'ДВИЖЕНИЕ + ПОВОРОТ':'ДВИЖЕНИЕ + БОКОВОЙ ХОД';
});
gaitSelect.addEventListener('change',()=>pilot?.setGait(gaitSelect.value));
tempo.addEventListener('input',()=>{if(pilot)pilot.setTempo(Number(tempo.value));rate.textContent=Number(tempo.value).toLocaleString('ru-RU')+'×';});
for(const {key,el,out} of poseSliders){el.addEventListener('input',()=>{
 const val=Number(el.value),denom=key==='height'?20:15;
 out.textContent=val+(key==='height'?' мм':'°');pilot?.setBody(key,val/denom);
});}
for(const el of [steerSelect,rightSelect,gaitSelect,tempo,...poseSliders.map(x=>x.el)]){
 for(const event of ['pointerdown','pointermove','click'])el.addEventListener(event,e=>e.stopPropagation());
}
addEventListener('keydown',e=>{const k=e.key.toLowerCase();if((('wasdqe'.includes(k)&&k.length===1)||['arrowup','arrowdown','arrowleft','arrowright'].includes(k))&&!['INPUT','SELECT'].includes(document.activeElement?.tagName)){keys.add(k);e.preventDefault();}});
addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));
addEventListener('blur',neutralize);
document.addEventListener('visibilitychange',()=>{if(document.hidden){neutralize();prev=null;}});
function input(){
 const dir=(hi,lo)=>(keys.has(hi)?1:0)-(keys.has(lo)?1:0);
 const drive=walkingInput(sticks.left,steerSelect.value,{
  forward:dir('w','s'), turn:dir('d','a'),strafe:dir('d','a')
 });
 // Q/E can optionally strafe when steering, or rotate when strafing.
 if(steerSelect.value==='turn')drive.left=dir('q','e');
 else drive.turn=dir('e','q');
 let right=rightInput(sticks.right,rightSelect.value);
 if(!right){
  const hx=dir('arrowright','arrowleft'),hy=dir('arrowup','arrowdown');
  if(hx||hy)right=rightInput({pointer:-1,x:hx,y:hy},rightSelect.value);
 }
 return {drive,right};
}
function tick(dt){
 const i=input();pilot.drive(i.drive.forward,i.drive.left,i.drive.turn);pilot.setRight(i.right);
 pose=pilot.tick(dt);applyPose(pose);seconds+=dt;
 if(seconds>.12){seconds=0;
  const g=factory.gaits[pose.gait],v=g.factory_full_stroke_mm/g.cycle_s*pose.tempo;
  stats.textContent=GAITS[pose.gait]+' · '+({tilt:'Корпус',shift:'Смещение',head:'Голова'}[rightSelect.value]);
  readout.textContent=`Заводские Tripod/Wave · подъём 40 мм · ${v.toFixed(0)} мм/с (команда)`;
  odom.textContent=`Опоры ${groundInfo?.stanceCount??0}/6 · смещение опор ${groundInfo?.maxSlipMm?.toFixed(1)??"—"} мм · курс ${((groundLock?.pose.yaw||0)*180/Math.PI).toFixed(1)}°`;
  if(pose.clamped)stats.textContent+=' · ОГРАНИЧЕНИЕ ПОЗЫ CAD';
   if((groundInfo?.maxSlipMm??0)>4||groundInfo?.stanceCount<3)stats.textContent+=' · ОПОРЫ НЕ СОГЛАСОВАНЫ';
 }
}
async function boot(){try{
 const [res,fres,hres,pres]=await Promise.all([
  fetch('../data/live-v4-gaits-kit.json',{cache:'no-store'}),
  fetch('../data/freenove-original-v7-kit.json',{cache:'no-store'}),
  fetch('../data/live-v5-motion-kit.json',{cache:'no-store'}),
  fetch('./planted-body-yaw-kit.json',{cache:'no-store'})
 ]);
 if(!res.ok||!fres.ok||!hres.ok||!pres.ok)throw Error('Не найдены офлайн-таблицы: '+res.status+'/'+fres.status+'/'+hres.status+'/'+pres.status);
 ref=await res.json();factory=await fres.json();headKit=await hres.json();plantedKit=await pres.json();validateFactoryKit(factory,ref);
 pilot=new PlantedCombinedPilot(factory,ref,headKit,plantedKit);groundLock=new GroundLock(plantedKit);pilot.setGait(gaitSelect.value);rightLabel();
 await loadCAD();pose=pilot.tick(0);applyPose(pose);cameraReset();status.hidden=true;ready=true;
 stats.textContent='Готово: кинематика с закреплением опор';
}catch(e){status.textContent='Невозможно запустить: '+e.message;console.error(e);}}
function animate(t){requestAnimationFrame(animate);
 if(ready&&prev!==null&&document.visibilityState==='visible')tick(Math.max(0,Math.min(.05,(t-prev)/1000)));
 prev=t;orbit.update();renderer.render(scene,camera);
}
boot();requestAnimationFrame(animate);
