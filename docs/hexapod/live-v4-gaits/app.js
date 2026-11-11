import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {Pilot,validateKit} from './kinematic-controller.js';

// This is the EXISTING Hexapod Lab 3D player, not a second physics viewer.
// The pilot applies joint-space trajectories compiled offline from the CAD30
// reference pose. No online MuJoCo/Drake solver, remote physics or fake Fz.
const viewer=document.getElementById('viewer');
const readout=document.getElementById('pilot-state'),velText=document.getElementById('pilot-speed-readout');
const pathText=document.getElementById('pilot-odom'),status=document.getElementById('pilot-load');
const pad=document.getElementById('pilot-pad'),thumb=document.getElementById('pilot-thumb');
const gain=document.getElementById('pilot-gain'),rateLabel=document.getElementById('pilot-rate');
const gaitSelect=document.getElementById('pilot-gait'),strideSelect=document.getElementById('pilot-stride');
const scene=new THREE.Scene();scene.background=new THREE.Color('#101c27');
const camera=new THREE.PerspectiveCamera(43,1,.01,30);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.08;
orbit.minDistance=.27;orbit.maxDistance=5;orbit.maxPolarAngle=Math.PI*.49;
scene.add(new THREE.HemisphereLight(0xd7e9f8,0x435461,2.4));
const sun=new THREE.DirectionalLight(0xffffff,2.2);sun.position.set(-1.5,3.7,2.6);scene.add(sun);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(50,50),new THREE.MeshStandardMaterial({color:0x152533,roughness:1}));
ground.rotation.x=-Math.PI/2;ground.position.y=-.006;scene.add(ground);
const grid=new THREE.GridHelper(10,100,0x5b8794,0x28424c);grid.position.y=.002;scene.add(grid);
// MuJoCo Z is up, Three.js Y is up.
const physics=new THREE.Group();physics.rotation.x=-Math.PI/2;scene.add(physics);
const cadRoot=new THREE.Group();physics.add(cadRoot);
const vec=(a)=>new THREE.Vector3(a[0],a[1],a[2]);
const quat=(q)=>new THREE.Quaternion(q[1],q[2],q[3],q[0]);
const zaxis=new THREE.Vector3(0,0,1);
let kit=null,pilot=null,rig=null,bodyNodes=new Map(),ready=false,prev=null;
let lastCenter=new THREE.Vector3(),lastReadout=0,pose=null;
let joystick={x:0,y:0,pointer:null},turn=0,turnHolds=new Set();
const keys=new Set();
const zero=()=>{joystick.x=0;joystick.y=0;joystick.pointer=null;turn=0;turnHolds.clear();thumb.style.left='50%';thumb.style.top='50%';document.querySelectorAll('.pilot-turn').forEach(b=>b.classList.remove('is-held'));if(pilot)pilot.stop();};
const stopPress=e=>{e.stopPropagation();};

function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function resetCamera(){
 if(!pilot||!kit)return;
 const root=kit.root_position;
 const center=new THREE.Vector3(root[0]+pilot.x,root[2],-(root[1]+pilot.y));
 const k=Math.min(2,Math.max(1,1/Math.max(.52,viewer.clientWidth/Math.max(1,viewer.clientHeight))));
 orbit.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(.53,.36,.64).multiplyScalar(k));lastCenter.copy(center);orbit.update();
}
document.getElementById('reset-camera').addEventListener('click',e=>{e.stopPropagation();resetCamera();});
function material(rgba){return new THREE.MeshStandardMaterial({color:new THREE.Color(...rgba.slice(0,3)),metalness:.20,roughness:.6,side:THREE.DoubleSide,transparent:rgba[3]<.995,opacity:rgba[3],depthWrite:rgba[3]>=.995});}
function placeholder(g){if(g.kind==='cylinder'){
 const geo=new THREE.CylinderGeometry(g.size[0],g.size[0],2*g.size[1],16);geo.rotateX(Math.PI/2);return geo;}
 return new THREE.BoxGeometry(...g.size.map(x=>2*x));}
async function loadCAD(){
 const response=await fetch('../data/cad-rig-live-v1-slow.json',{cache:'no-store'});
 if(!response.ok)throw Error('CAD30 metadata HTTP '+response.status);
 rig=await response.json();
 if(rig.format!=='fnk0052-mujoco-cad-visual-replay-v1'||rig.meshes.length!==30||rig.bodies.length!==27||rig.frame_stride!==189)throw Error('CAD30 metadata mismatched');
 for(const b of rig.bodies){const group=new THREE.Group();group.name=b.name;cadRoot.add(group);bodyNodes.set(b.id,group);}
 const loader=new STLLoader(),geomById=new Map();const errors=[];let next=0,loaded=0;
 async function worker(){while(next<rig.meshes.length){const mesh=rig.meshes[next++];
  try{const geo=await loader.loadAsync(mesh.url);geo.translate(...mesh.mesh_pos.map(x=>-x));geo.applyQuaternion(quat(mesh.mesh_quat_wxyz).invert());geomById.set(mesh.id,geo);loaded++;status.textContent=`Геометрия Freenove ${loaded}/30 ...`;}catch(err){errors.push(mesh.filename+': '+err.message);}
 }}
 await Promise.all(Array.from({length:5},()=>worker()));
 const materials=new Map();
 for(const g of rig.geoms){const group=bodyNodes.get(g.body);if(!group)throw Error('Missing CAD body '+g.body);
  if(!materials.has(g.rgba.join(',')))materials.set(g.rgba.join(','),material(g.rgba));
  const shape=g.kind==='mesh'?geomById.get(g.mesh):placeholder(g);if(!shape)continue;
  const child=new THREE.Group();child.position.copy(vec(g.pos));child.quaternion.copy(quat(g.quat_wxyz));child.add(new THREE.Mesh(shape,materials.get(g.rgba.join(','))));group.add(child);
 }
 if(loaded<30)throw Error(`CAD30 assets ${loaded}/30 недоступны; проверь кэш/STL. ${errors.slice(0,2).join('; ')}`);
}
function applyPose(s){
 const rootP=vec(kit.root_position).add(new THREE.Vector3(s.x,s.y,0));
 const yawQuat=new THREE.Quaternion().setFromAxisAngle(zaxis,s.yaw);
 const rootQ=yawQuat.clone().multiply(quat(kit.root_quat_wxyz));
 const frames=new Map();frames.set(1,{p:rootP,q:rootQ});
 for(const n of kit.skeleton){
  const parent=frames.get(n.parent);if(!parent)throw Error('Bad FK parent '+n.parent);
  const p=vec(n.p),q=quat(n.q);
  if(n.adr!==undefined){
    const anchor=vec(n.anchor),axis=vec(n.axis),dtheta=s.q[n.adr]-kit.q_stand[n.adr];
    const dq=new THREE.Quaternion().setFromAxisAngle(axis,dtheta);
    p.sub(anchor).applyQuaternion(dq).add(anchor);q.premultiply(dq);
  }
  p.applyQuaternion(parent.q).add(parent.p);q.premultiply(parent.q);
  frames.set(n.id,{p,q});
 }
 for(const [id,group] of bodyNodes){const frame=frames.get(id);if(frame){group.position.copy(frame.p);group.quaternion.copy(frame.q);}}
 const center=new THREE.Vector3(rootP.x,rootP.z,-rootP.y);
 const d=center.clone().sub(lastCenter);camera.position.add(d);orbit.target.add(d);lastCenter.copy(center);
}
function inputFromControls(){
 const keyV=(hi,lo)=>(keys.has(hi)?1:0)-(keys.has(lo)?1:0);
 const vForward=joystick.pointer!==null?joystick.y:keyV('w','s');
 const vLeft=joystick.pointer!==null?-joystick.x:keyV('a','d');
 const vTurn=(turn!==0)?turn:keyV('q','e');
 return {vx:vForward,vy:vLeft,wz:vTurn};
}
function pilotTick(dt){
 const input=inputFromControls();pilot.command(input.vx,input.vy,input.wz);
 pose=pilot.tick(dt);applyPose(pose);
 lastReadout+=dt;
 if(lastReadout>.1){
  lastReadout=0;const {vx,vy,wz}=pose.command;
  const label={wave:'Wave',ripple:'Ripple',tripod:'Tripod'};
  const switching=pose.gait!==pose.pendingGait?' → '+label[pose.pendingGait]:'';
  readout.textContent=`${label[pose.gait]}${switching} · ${pose.strideMm.toFixed(0)} мм${pose.moving?'':' · стойка'}`;
  const linear=pose.strideMm/kit.period_s*pose.tempo;
  const omega=pose.strideMm/1000/kit.nominal_turn_radius_m/kit.period_s*pose.tempo*180/Math.PI;
  velText.textContent=`Команда: vx ${(vx*linear).toFixed(1)} · vy ${(vy*linear).toFixed(1)} мм/с · ω ${(wz*omega).toFixed(1)}°/с`;
  pathText.textContent=`Расчётный путь: ${(Math.hypot(pilot.x,pilot.y)*100).toFixed(1)} см · ${(pilot.yaw*180/Math.PI).toFixed(1)}°`;
 }
}
function handleStick(e){
 const box=pad.getBoundingClientRect(),radius=Math.min(box.width,box.height)*.38;
 let x=(e.clientX-(box.left+box.width/2))/radius, y=-(e.clientY-(box.top+box.height/2))/radius;
 const n=Math.hypot(x,y);if(n>1){x/=n;y/=n;}if(Math.hypot(x,y)<.10){x=0;y=0;}
 joystick.x=x;joystick.y=y;thumb.style.left=`${50+x*38}%`;thumb.style.top=`${50-y*38}%`;
}
pad.addEventListener('pointerdown',e=>{e.preventDefault();e.stopPropagation();if(joystick.pointer!==null)return;joystick.pointer=e.pointerId;pad.setPointerCapture(e.pointerId);handleStick(e);});
pad.addEventListener('pointermove',e=>{e.preventDefault();e.stopPropagation();if(e.pointerId===joystick.pointer)handleStick(e);});
function releasePad(e){e.preventDefault();e.stopPropagation();if(e.pointerId!==joystick.pointer)return;joystick.pointer=null;joystick.x=0;joystick.y=0;thumb.style.left='50%';thumb.style.top='50%';}
pad.addEventListener('pointerup',releasePad);pad.addEventListener('pointercancel',releasePad);pad.addEventListener('lostpointercapture',e=>{if(e.pointerId===joystick.pointer){joystick.pointer=null;joystick.x=joystick.y=0;thumb.style.left='50%';thumb.style.top='50%';}});
for(const [id,v] of [['pilot-ccw',1],['pilot-cw',-1]]){
 const b=document.getElementById(id);
 b.addEventListener('pointerdown',e=>{e.preventDefault();e.stopPropagation();b.setPointerCapture(e.pointerId);turnHolds.add(id);turn=v;b.classList.add('is-held');});
 const clear=e=>{e.preventDefault();e.stopPropagation();turnHolds.delete(id);b.classList.remove('is-held');turn=turnHolds.has('pilot-ccw')?1:turnHolds.has('pilot-cw')?-1:0;};
 b.addEventListener('pointerup',clear);b.addEventListener('pointercancel',clear);b.addEventListener('lostpointercapture',clear);
}
document.getElementById('pilot-stop').addEventListener('click',e=>{e.stopPropagation();zero();keys.clear();});
function updateTempo(){if(!pilot)return;pilot.setTempo(Number(gain.value));rateLabel.textContent=Number(gain.value).toLocaleString('ru-RU')+'×';}
gaitSelect.addEventListener('change',()=>{if(pilot)pilot.setGait(gaitSelect.value);});
strideSelect.addEventListener('change',()=>{if(pilot)pilot.setStride(Number(strideSelect.value));});
for(const el of [gaitSelect,strideSelect]){el.addEventListener('pointerdown',stopPress);el.addEventListener('click',stopPress);}
gain.addEventListener('input',updateTempo);
gain.addEventListener('change',updateTempo);
gain.addEventListener('pointerdown',stopPress);gain.addEventListener('pointermove',stopPress);gain.addEventListener('click',stopPress);
addEventListener('keydown',e=>{const k=e.key.toLowerCase();if('123'.includes(k)&&k.length===1){gaitSelect.value=({1:'wave',2:'ripple',3:'tripod'})[k];gaitSelect.dispatchEvent(new Event('change'));return;}if('wasdqe'.includes(k)&&k.length===1&&!['INPUT','SELECT'].includes(document.activeElement?.tagName)){keys.add(k);e.preventDefault();}});
addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));
addEventListener('blur',()=>{zero();keys.clear();});
document.addEventListener('visibilitychange',()=>{if(document.hidden){zero();keys.clear();prev=null;}});
async function boot(){
 try{
  const response=await fetch('../data/live-v4-gaits-kit.json',{cache:'no-store'});
  if(!response.ok)throw Error('Контроллер HTTP '+response.status);
  kit=await response.json();validateKit(kit);pilot=new Pilot(kit);pilot.setGait(gaitSelect.value);pilot.setStride(Number(strideSelect.value));updateTempo();
  await loadCAD();pose=pilot.tick(0);applyPose(pose);resetCamera();
  readout.textContent='Готово · джойстик';status.hidden=true;ready=true;
 }catch(err){status.textContent='Движение недоступно: '+err.message;readout.textContent='Ошибка набора';console.error(err);}
}
function animate(t){requestAnimationFrame(animate);
 if(ready){if(prev!==null&&document.visibilityState==='visible')pilotTick(Math.max(0,Math.min(.1,(t-prev)/1000)));}
 prev=t;orbit.update();renderer.render(scene,camera);
}
boot();requestAnimationFrame(animate);
