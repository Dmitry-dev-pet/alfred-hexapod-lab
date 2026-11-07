import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {FactoryPilot,GAITS,validateFactoryKit} from './factory-controller.js';

// Freenove manufacturer's *command amplitudes*, mapped offline to the owner's
// real CAD30 joints. Rendering is only FK+interpolation (no runtime MuJoCo).
const viewer=document.getElementById('viewer');
const status=document.getElementById('pilot-load');
const stats=document.getElementById('pilot-state');
const readout=document.getElementById('pilot-speed-readout');
const odom=document.getElementById('pilot-odom');
const pad=document.getElementById('pilot-pad'),thumb=document.getElementById('pilot-thumb');
const modeSelect=document.getElementById('factory-mode'),gaitSelect=document.getElementById('factory-gait');
const tempo=document.getElementById('factory-tempo'),rate=document.getElementById('factory-rate');
const poseSliders=[...['height','roll','pitch','yaw'].map(key=>({key,el:document.getElementById('pose-'+key),out:document.getElementById('pose-'+key+'-value')}))];
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
let ref=null,factory=null,pilot=null,ready=false,pose=null,prev=null,seconds=0;
let bodyNodes=new Map(),cameraCenter=new THREE.Vector3();
let stick={x:0,y:0,pointer:null},turn=0,turnHeld=new Set();
const keys=new Set();
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function cameraReset(){
 if(!factory||!pilot)return;
 const p=vec(factory.cad_adapter.neutral_root_xyz_m);
 p.x+=pilot.x;p.y+=pilot.y;
 const offset=pose?.rootOffsetWorld||[0,0,0];p.add(vec(offset));
 const c=new THREE.Vector3(p.x,p.z,-p.y);
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
function applyPose(s){
 const root=vec(factory.cad_adapter.neutral_root_xyz_m).add(new THREE.Vector3(s.x,s.y,0)).add(vec(s.rootOffsetWorld));
 const yawQ=new THREE.Quaternion().setFromAxisAngle(zaxis,s.yaw);
 const r=vec(s.bodyRotvecWorld),a=r.length(),attitude=a>1e-9?new THREE.Quaternion().setFromAxisAngle(r.clone().divideScalar(a),a):new THREE.Quaternion();
 const rotation=attitude.multiply(yawQ).multiply(quat(ref.root_quat_wxyz));
 const frames=new Map();frames.set(1,{p:root,q:rotation});
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
 for(const [id,obj] of bodyNodes){const f=frames.get(id);if(f){obj.position.copy(f.p);obj.quaternion.copy(f.q);}}
 const c=new THREE.Vector3(root.x,root.z,-root.y);const delta=c.clone().sub(cameraCenter);
 camera.position.add(delta);orbit.target.add(delta);cameraCenter.copy(c);
}
function positionStick(e){
 const r=pad.getBoundingClientRect(),rad=Math.min(r.width,r.height)*.38;
 let x=(e.clientX-r.left-r.width/2)/rad,y=-(e.clientY-r.top-r.height/2)/rad;
 const l=Math.hypot(x,y);if(l>1){x/=l;y/=l;}if(Math.hypot(x,y)<.10){x=y=0;}
 stick.x=x;stick.y=y;thumb.style.left=`${50+x*38}%`;thumb.style.top=`${50-y*38}%`;
}
function clearStick(){stick.x=stick.y=0;stick.pointer=null;thumb.style.left='50%';thumb.style.top='50%';}
pad.addEventListener('pointerdown',e=>{e.preventDefault();e.stopPropagation();if(stick.pointer!==null)return;stick.pointer=e.pointerId;pad.setPointerCapture(e.pointerId);positionStick(e);});
pad.addEventListener('pointermove',e=>{e.preventDefault();e.stopPropagation();if(e.pointerId===stick.pointer)positionStick(e);});
function releaseStick(e){e.preventDefault();e.stopPropagation();if(e.pointerId===stick.pointer)clearStick();}
pad.addEventListener('pointerup',releaseStick);pad.addEventListener('pointercancel',releaseStick);
pad.addEventListener('lostpointercapture',e=>{if(e.pointerId===stick.pointer)clearStick();});
for(const [id,sign] of [['pilot-ccw',1],['pilot-cw',-1]]){
 const b=document.getElementById(id);
 b.addEventListener('pointerdown',e=>{e.preventDefault();e.stopPropagation();b.setPointerCapture(e.pointerId);turnHeld.add(id);turn=sign;b.classList.add('is-held');});
 const release=e=>{e.preventDefault();e.stopPropagation();turnHeld.delete(id);turn=turnHeld.has('pilot-ccw')?1:turnHeld.has('pilot-cw')?-1:0;b.classList.remove('is-held');};
 b.addEventListener('pointerup',release);b.addEventListener('pointercancel',release);b.addEventListener('lostpointercapture',release);
}
function neutralize(){clearStick();turn=0;turnHeld.clear();keys.clear();document.querySelectorAll('.pilot-turn').forEach(n=>n.classList.remove('is-held'));
 for(const {key,el,out} of poseSliders){el.value='0';out.textContent='0'+(key==='height'?' мм':'°');}
 pilot?.stop();}
document.getElementById('factory-stop').addEventListener('click',e=>{e.stopPropagation();neutralize();});
modeSelect.addEventListener('change',()=>{
 neutralize();pilot?.setMode(modeSelect.value);
 document.querySelectorAll('.factory-panel').forEach(p=>p.hidden=p.dataset.mode!==modeSelect.value);
 document.getElementById('pad-label').textContent=modeSelect.value==='walk'?'ХОДЬБА':'СМЕЩЕНИЕ КОРПУСА';
});
gaitSelect.addEventListener('change',()=>pilot?.setGait(gaitSelect.value));
tempo.addEventListener('input',()=>{if(pilot)pilot.setTempo(Number(tempo.value));rate.textContent=Number(tempo.value).toLocaleString('ru-RU')+'×';});
for(const {key,el,out} of poseSliders){el.addEventListener('input',()=>{const val=Number(el.value),denom=key==='height'?20:15;out.textContent=val+(key==='height'?' мм':'°');pilot?.setBody(key,val/denom);});}
for(const el of [modeSelect,gaitSelect,tempo,...poseSliders.map(x=>x.el)]){
 for(const event of ['pointerdown','pointermove','click'])el.addEventListener(event,e=>e.stopPropagation());
}
addEventListener('keydown',e=>{const k=e.key.toLowerCase();if('wasdqe'.includes(k)&&k.length===1&&!['INPUT','SELECT'].includes(document.activeElement?.tagName)){keys.add(k);e.preventDefault();}});
addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));
addEventListener('blur',neutralize);
document.addEventListener('visibilitychange',()=>{if(document.hidden){neutralize();prev=null;}});
function input(){const dir=(hi,lo)=>(keys.has(hi)?1:0)-(keys.has(lo)?1:0);
 return {forward:stick.pointer!==null?stick.y:dir('w','s'),left:stick.pointer!==null?-stick.x:dir('a','d'),turn:turn||dir('q','e')};}
function tick(dt){
 const i=input();if(modeSelect.value==='walk')pilot.command(i.forward,i.left,i.turn);
 else {pilot.setBody('forward',i.forward);pilot.setBody('left',i.left);}
 pose=pilot.tick(dt);applyPose(pose);seconds+=dt;
 if(seconds>.12){seconds=0;
  if(pose.mode==='walk'){
   const g=factory.gaits[pose.gait],v=g.factory_full_stroke_mm/g.cycle_s*pose.tempo;
   stats.textContent=GAITS[pose.gait];
   readout.textContent=`Заводская команда ±35 мм · подъём 40 мм · ${v.toFixed(0)} мм/с (задано)`;
   odom.textContent=`Расчётный путь ${(Math.hypot(pose.x,pose.y)*100).toFixed(1)} см · поворот ${(pose.yaw*180/Math.PI).toFixed(1)}°`;
  }else{
   stats.textContent='CMD_POSITION + CMD_ATTITUDE';
   readout.textContent=`Корпус: ${(pose.body.forward*40).toFixed(0)} / ${(pose.body.left*40).toFixed(0)} мм; высота ${(pose.body.height*20).toFixed(0)} мм`;
   odom.textContent=`Крен ${(pose.body.roll*15).toFixed(0)}° · тангаж ${(pose.body.pitch*15).toFixed(0)}° · поворот ${(pose.body.yaw*15).toFixed(0)}°`;
  }
  if(pose.clamped)stats.textContent+=' · ДОСТИЖЕНИЕ ПРЕДЕЛА CAD';
 }
}
async function boot(){try{
 const [res,fres]=await Promise.all([fetch('../data/live-v4-gaits-kit.json',{cache:'no-store'}),fetch('../data/freenove-original-v7-kit.json',{cache:'no-store'})]);
 if(!res.ok||!fres.ok)throw Error('Не найдены офлайн-таблицы: '+res.status+'/'+fres.status);
 ref=await res.json();factory=await fres.json();validateFactoryKit(factory,ref);pilot=new FactoryPilot(factory,ref);pilot.setGait(gaitSelect.value);
 await loadCAD();pose=pilot.tick(0);applyPose(pose);cameraReset();status.hidden=true;ready=true;
 stats.textContent='Готово: Freenove Original · 2 походки';
}catch(e){status.textContent='Невозможно запустить: '+e.message;console.error(e);}}
function animate(t){requestAnimationFrame(animate);
 if(ready&&prev!==null&&document.visibilityState==='visible')tick(Math.max(0,Math.min(.05,(t-prev)/1000)));
 prev=t;orbit.update();renderer.render(scene,camera);
}
boot();requestAnimationFrame(animate);
