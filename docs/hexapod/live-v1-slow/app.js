import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// These frames are physical qpos/Fz from the live-velocity test: recorded replay, NOT live joystick.
const caseId='live-v1-slow';
const viewer=document.getElementById('viewer');
const seek=document.getElementById('seek'),play=document.getElementById('play');
const speed=document.getElementById('speed'),clock=document.getElementById('clock');
const feetLabel=document.getElementById('feet'),phaseLabel=document.getElementById('phase-label');
const supportLabel=document.getElementById('support-margin'),distanceLabel=document.getElementById('distance');
const cadStatus=document.getElementById('cad-status'),errorLabel=document.getElementById('error');
const follow=document.getElementById('follow'),contacts=document.getElementById('contacts');
const pathToggle=document.getElementById('path'),cadToggle=document.getElementById('show-cad');
const scene=new THREE.Scene();scene.background=new THREE.Color('#111c2a');
const camera=new THREE.PerspectiveCamera(43,1,.01,25);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.6));renderer.outputColorSpace=THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.075;orbit.minDistance=.25;orbit.maxDistance=4;orbit.maxPolarAngle=Math.PI*.495;
scene.add(new THREE.HemisphereLight(0xd1e8ff,0x4b555c,2.6));
const sun=new THREE.DirectionalLight(0xffffff,2.2);sun.position.set(-1.4,3.7,2.5);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(25,25),new THREE.MeshStandardMaterial({color:0x142231,roughness:.88}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.005;scene.add(floor);
const grid=new THREE.GridHelper(8,80,0x547086,0x293d4c);grid.position.y=.001;scene.add(grid);
// MuJoCo Z up; camera and floor Three.js Y up.
const physics=new THREE.Group();physics.rotation.x=-Math.PI/2;scene.add(physics);
const cadRoot=new THREE.Group();physics.add(cadRoot);
const markers=new THREE.Group();physics.add(markers);
const markerGeometry=new THREE.SphereGeometry(.014,12,8);
const contactMaterial=new THREE.MeshStandardMaterial({color:0x65e4ad,emissive:0x12432e});
const airborneMaterial=new THREE.MeshStandardMaterial({color:0xefad5f,emissive:0x513517});
const toes=Array.from({length:6},()=>{
  const m=new THREE.Mesh(markerGeometry,airborneMaterial);markers.add(m);return m;
});
let rig=null,observed=null,bodies=[],cadLoaded=0,cadReady=false,played=true,t=0,prev=null,pathLine=null;
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
const lastCenter=new THREE.Vector3();
const phases={forward:'Команда: вперёд',left:'Команда: влево',turn:'Команда: поворот против часовой',stop:'Команда: остановка'};
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);
  camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function poseAt(time){return Math.min(observed.frames.length-1,Math.max(0,time/observed.sample_interval_s));}
function lerpRow(a,b,ix,f){return a[ix]*(1-f)+b[ix]*f;}
function centerOf(frame){return new THREE.Vector3(frame[0],frame[2],-frame[1]);}
function resetCamera(){if(!observed)return;
  const i=Math.min(observed.frames.length-1,Math.round(poseAt(t)));
  const c=centerOf(observed.frames[i]);orbit.target.copy(c);
  const ratio=viewer.clientWidth/Math.max(1,viewer.clientHeight);const k=ratio<.9?Math.min(1.85,.9/ratio):1;
  camera.position.copy(c).add(new THREE.Vector3(.49,.34,.63).multiplyScalar(k));lastCenter.copy(c);orbit.update();}
function wxyz(q){return new THREE.Quaternion(q[1],q[2],q[3],q[0]);}
function cadMaterial(rgba){return new THREE.MeshStandardMaterial({color:new THREE.Color(rgba[0],rgba[1],rgba[2]),metalness:.22,roughness:.58,side:THREE.DoubleSide,transparent:rgba[3]<.995,opacity:rgba[3],depthWrite:rgba[3]>=.995});}
function fallback(g){if(g.kind==='cylinder'){const geo=new THREE.CylinderGeometry(g.size[0],g.size[0],2*g.size[1],16);geo.rotateX(Math.PI/2);return geo;}
  return new THREE.BoxGeometry(2*g.size[0],2*g.size[1],2*g.size[2]);}
async function loadCAD(){
  try{
    const res=await fetch(`../data/cad-rig-${caseId}.json`,{cache:'no-store'});
    if(!res.ok)throw Error('CAD data HTTP '+res.status);
    rig=await res.json();
    if(rig.format!=='fnk0052-mujoco-cad-visual-replay-v1'||rig.meshes.length!==30||rig.bodies.length!==27||rig.frame_stride!==189||rig.samples<100||rig.frames.length!==rig.samples)throw Error('Invalid CAD30 replay');
    const bodyMap=new Map();
    bodies=rig.bodies.map(b=>{const g=new THREE.Group();g.name=b.name;cadRoot.add(g);bodyMap.set(b.id,g);return g;});
    const meshGeometry=new Map(),loader=new STLLoader();let next=0;const errors=[];
    async function worker(){while(next<rig.meshes.length){const mesh=rig.meshes[next++];
      try{const geo=await loader.loadAsync(mesh.url);geo.translate(-mesh.mesh_pos[0],-mesh.mesh_pos[1],-mesh.mesh_pos[2]);geo.applyQuaternion(wxyz(mesh.mesh_quat_wxyz).invert());meshGeometry.set(mesh.id,geo);cadLoaded++;cadStatus.textContent=`CAD: ${cadLoaded}/30 STL`;}catch(e){errors.push(mesh.filename+': '+e.message);}}}
    await Promise.all(Array.from({length:5},()=>worker()));
    const mats=new Map();
    for(const g of rig.geoms){const parent=bodyMap.get(g.body);if(!parent)throw Error('Missing CAD body '+g.body);
      const local=new THREE.Group();local.position.set(...g.pos);local.quaternion.copy(wxyz(g.quat_wxyz));
      const mk=g.rgba.join(',');if(!mats.has(mk))mats.set(mk,cadMaterial(g.rgba));
      const geom=g.kind==='mesh'?meshGeometry.get(g.mesh):fallback(g);if(!geom)continue;
      local.add(new THREE.Mesh(geom,mats.get(mk)));parent.add(local);}
    cadReady=true;cadRoot.visible=cadToggle.checked;
    cadStatus.textContent=cadLoaded===30?'Оригинальный CAD: 30/30 STL · 2 детали заменены':`Оригинальный CAD: ${cadLoaded}/30 STL · ${errors.length} ошибок`;
    if(errors.length)console.warn('Missing CAD meshes:',errors);
    if(observed)applyFrame(t);
  }catch(e){cadStatus.textContent='CAD недоступен: '+e.message;console.warn(e);}
}
function applyFrame(time){if(!observed)return;
  const at=poseAt(time),ia=Math.floor(at),ib=Math.min(ia+1,observed.frames.length-1),f=at-ia;
  const a=observed.frames[ia],b=observed.frames[ib];
  const nowCenter=new THREE.Vector3(lerpRow(a,b,0,f),lerpRow(a,b,2,f),-lerpRow(a,b,1,f));
  if(follow.checked){const shift=nowCenter.clone().sub(lastCenter);camera.position.add(shift);orbit.target.add(shift);}lastCenter.copy(nowCenter);
  const idx=Math.min(observed.frames.length-1,Math.round(at));const sample=observed.support_samples[idx];
  const mask=observed.frames[idx][79];
  feetLabel.textContent=`${sample.contacts} / 6`;
  for(let i=0;i<6;i++){const off=7+i*12+9;
    toes[i].position.set(lerpRow(a,b,off,f),lerpRow(a,b,off+1,f),lerpRow(a,b,off+2,f));
    toes[i].material=mask&(1<<i)?contactMaterial:airborneMaterial;}
  markers.visible=contacts.checked;
  const fwd=observed.forward_xy,init=observed.frames[0];
  const dist=((nowCenter.x-init[0])*fwd[0]+(-nowCenter.z-init[1])*fwd[1])*100;
  distanceLabel.textContent=dist.toLocaleString('ru-RU',{minimumFractionDigits:1,maximumFractionDigits:1});
  phaseLabel.textContent=phases[observed.phase_samples[idx]]||observed.phase_samples[idx];
  const fz=sample.fz_N;
  supportLabel.textContent=`Фактические силы: ${['ml','mr','bl','br','fr','fl'].map(k=>k+' '+fz[k].toFixed(2)+' Н').join(' · ')} · Направление: ${sample.root_forward_mm.toFixed(1)} / ${sample.root_left_mm.toFixed(1)} мм · Поворот ${(sample.yaw_rad*180/Math.PI).toFixed(1)}°`;
  if(cadReady){const x=rig.frames[ia],y=rig.frames[ib];
    for(let i=0;i<bodies.length;i++){const off=i*7,o=bodies[i];
      o.position.set(lerpRow(x,y,off,f),lerpRow(x,y,off+1,f),lerpRow(x,y,off+2,f));
      qa.set(x[off+4],x[off+5],x[off+6],x[off+3]);
      qb.set(y[off+4],y[off+5],y[off+6],y[off+3]);o.quaternion.copy(qa).slerp(qb,f);}}
  clock.textContent=String(Math.floor(time/60)).padStart(2,'0')+':'+String(Math.floor(time%60)).padStart(2,'0');
  seek.value=String(time);seek.style.setProperty('--pct',`${time/observed.duration_s*100}%`);
}
function togglePlay(yes){played=yes;play.textContent=played?'Ⅱ':'▶';play.setAttribute('aria-label',played?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>togglePlay(!played));
seek.addEventListener('input',()=>{t=Number(seek.value);applyFrame(t);});
contacts.addEventListener('change',()=>applyFrame(t));
pathToggle.addEventListener('change',()=>{if(pathLine)pathLine.visible=pathToggle.checked;});
cadToggle.addEventListener('change',()=>cadRoot.visible=cadReady&&cadToggle.checked);
document.getElementById('reset-camera').addEventListener('click',resetCamera);
addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT'].includes(document.activeElement?.tagName)){e.preventDefault();togglePlay(!played);}});
async function load(){try{
  const response=await fetch(`../data/${caseId}-points.json`,{cache:'no-store'});
  if(!response.ok)throw Error('Physical data HTTP '+response.status);
  const obj=await response.json();
  if(obj.format!=='fnk0052-physical-points-v1'||obj.study!=='issue62-live-velocity-v1'||obj.case_id!==caseId||obj.stride!==80||obj.summary?.status!=='PHYSICAL_PASS'||obj.summary?.live_joystick!==false||obj.frames.length<100||obj.frames.length!==obj.sample_count||obj.support_samples.length!==obj.sample_count||obj.phase_samples.length!==obj.sample_count||!(obj.duration_s>12&&obj.duration_s<13))throw Error('Unverified MuJoCo recording');
  observed=obj;seek.max=String(obj.duration_s);
  const positions=obj.frames.map(f=>new THREE.Vector3(f[0],.006,-f[1]));
  pathLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints(positions),new THREE.LineBasicMaterial({color:0x63dcae,transparent:true,opacity:.7}));
  pathLine.visible=pathToggle.checked;scene.add(pathLine);resetCamera();applyFrame(0);
}catch(e){errorLabel.hidden=false;errorLabel.textContent='Не удалось загрузить реальную запись: '+e.message;togglePlay(false);}}
function animate(ts){requestAnimationFrame(animate);if(prev!==null&&played&&observed){t=(t+(ts-prev)/1000*Number(speed.value))%observed.duration_s;applyFrame(t);}
  prev=ts;orbit.update();renderer.render(scene,camera);}
load();loadCAD();requestAnimationFrame(animate);