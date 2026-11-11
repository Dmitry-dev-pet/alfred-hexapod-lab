import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {validatePack,frameAt,contactCount,LEGS,LABELS,VARIANTS} from './replay-controller.js';

// Replay of actual MuJoCo mj_step full qpos recordings. No command synth, online IK,
// welded feet, artificial ground lock, or dynamics integration on the tablet.
const el=id=>document.getElementById(id);
const viewer=el('viewer'),status=el('replay-status'),variantSelect=el('variant'),runSelect=el('experiment');
const slider=el('timeline'),clock=el('clock'),toggle=el('toggle'),pace=el('pace');
const contactLine=el('contact-line'),motionLine=el('motion-line'),slipLine=el('slip-line'),note=el('experiment-info');
const scene=new THREE.Scene();scene.background=new THREE.Color('#101b24');
const camera=new THREE.PerspectiveCamera(43,1,.01,30);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.075;
orbit.minDistance=.25;orbit.maxDistance=6;orbit.maxPolarAngle=Math.PI*.495;
scene.add(new THREE.HemisphereLight(0xd8ebf7,0x435561,2.6));
const sun=new THREE.DirectionalLight(0xffffff,2.25);sun.position.set(-1.5,3.5,2.6);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(50,50),new THREE.MeshStandardMaterial({color:0x14252e,roughness:1}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.004;scene.add(floor);
const grid=new THREE.GridHelper(8,80,0x527c88,0x28404c);grid.position.y=.002;scene.add(grid);
const physics=new THREE.Group();physics.rotation.x=-Math.PI/2;scene.add(physics);
const cadRoot=new THREE.Group();physics.add(cadRoot);
const vec=a=>new THREE.Vector3(a[0],a[1],a[2]);
const quat=a=>new THREE.Quaternion(a[1],a[2],a[3],a[0]);
const materials=new Map(),nodes=new Map(),dots=new Map();
let ref=null,rig=null,toeMeta=null,pack=null,run=null,seconds=0,playing=false,ready=false,prev=null,lastDisplay=-1,current=null,cameraCenter=new THREE.Vector3();
const cache=new Map();
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function mat(rgba){return new THREE.MeshStandardMaterial({color:new THREE.Color(...rgba.slice(0,3)),roughness:.56,metalness:.22,side:THREE.DoubleSide,transparent:rgba[3]<.995,opacity:rgba[3],depthWrite:rgba[3]>=.995});}
function proxy(g){if(g.kind==='cylinder'){const geo=new THREE.CylinderGeometry(g.size[0],g.size[0],2*g.size[1],16);geo.rotateX(Math.PI/2);return geo;}return new THREE.BoxGeometry(...g.size.map(v=>2*v));}
async function json(url){const res=await fetch(url,{cache:'no-store'});if(!res.ok)throw Error('Файл недоступен '+url+' ('+res.status+')');return res.json();}
async function prepareCAD(){
  [ref,rig,toeMeta]=await Promise.all([json('../data/live-v4-gaits-kit.json'),json('../data/cad-rig-live-v1-slow.json'),json('../freenove-grounded-v11/planted-body-yaw-kit.json')]);
  if(ref.format!=='fnk0052-cad30-kinematic-gaits-v4'||ref.skeleton.length!==46||rig.format!=='fnk0052-mujoco-cad-visual-replay-v1'||rig.meshes.length!==30||rig.bodies.length!==27||Object.keys(toeMeta.cad_tip_local||{}).length!==6)throw Error('CAD30 не соответствует физическим записям');
  for(const b of rig.bodies){const obj=new THREE.Group();obj.name=b.name;nodes.set(b.id,obj);cadRoot.add(obj);}
  const loader=new STLLoader(),meshes=new Map(),errors=[];let next=0,done=0;
  async function fetcher(){while(next<rig.meshes.length){const spec=rig.meshes[next++];try{const g=await loader.loadAsync(spec.url);g.translate(...spec.mesh_pos.map(x=>-x));g.applyQuaternion(quat(spec.mesh_quat_wxyz).invert());meshes.set(spec.id,g);status.textContent='3D CAD: '+(++done)+'/30';}catch(e){errors.push(spec.filename+': '+e.message);}}}
  await Promise.all(Array.from({length:5},fetcher));
  if(done!==30)throw Error('Нет локальных STL CAD30: '+errors.slice(0,2).join('; '));
  for(const g of rig.geoms){const target=nodes.get(g.body);if(!target)throw Error('Нет сегмента модели '+g.body);
    const mkey=g.rgba.join(',');if(!materials.has(mkey))materials.set(mkey,mat(g.rgba));
    const shape=g.kind==='mesh'?meshes.get(g.mesh):proxy(g);if(!shape)throw Error('Нет геометрии '+g.mesh);
    const group=new THREE.Group();group.position.copy(vec(g.pos));group.quaternion.copy(quat(g.quat_wxyz));group.add(new THREE.Mesh(shape,materials.get(mkey)));target.add(group);
  }
  for(const name of LEGS){const marker=new THREE.Mesh(new THREE.SphereGeometry(.007,12,8),new THREE.MeshBasicMaterial({color:0x526672,depthTest:false}));marker.renderOrder=9;physics.add(marker);dots.set(name,marker);}
}
function renderFrame(s){
 const root=vec(s.root),qroot=quat(s.quat),frames=new Map();frames.set(1,{p:root,q:qroot});
 for(const node of ref.skeleton){const parent=frames.get(node.parent);if(!parent)throw Error('CAD30: неверный скелет '+node.parent);
  const pos=vec(node.p),ori=quat(node.q);
  if(node.adr!==undefined){const anchor=vec(node.anchor),axis=vec(node.axis),delta=s.joints[node.adr]-ref.q_stand[node.adr];const dq=new THREE.Quaternion().setFromAxisAngle(axis,delta);pos.sub(anchor).applyQuaternion(dq).add(anchor);ori.premultiply(dq);}
  pos.applyQuaternion(parent.q).add(parent.p);ori.premultiply(parent.q);frames.set(node.id,{p:pos,q:ori});
 }
 for(const [id,obj] of nodes){const f=frames.get(id);if(f){obj.position.copy(f.p);obj.quaternion.copy(f.q);}}
 for(const [i,name] of LEGS.entries()){
  const info=toeMeta.cad_tip_local[name],parent=frames.get(info.body),dot=dots.get(name);
  dot.position.copy(vec(info.tip_local_xyz_m).applyQuaternion(parent.q).add(parent.p));
  const force=s.fz[i];dot.material.color.setHex(force>.5?0x41ef9d:force>.05?0xf5be56:0x657c89);
  dot.scale.setScalar(force>.5?1.15:1);
 }
 const center=new THREE.Vector3(root.x,root.z,-root.y),shift=center.clone().sub(cameraCenter);
 camera.position.add(shift);orbit.target.add(shift);cameraCenter.copy(center);
 current=s;
 if(Math.abs(seconds-lastDisplay)>.08||seconds===0||Math.abs(seconds-run.duration_s)<.001){
   const loaded=contactCount(s.contactMask);contactLine.textContent='Опоры: '+loaded+'/6 · Fz '+s.fz.map(v=>v.toFixed(1)).join(' / ')+' Н';
   motionLine.textContent='Корпус: z='+(s.root[2]*100).toFixed(1)+' см · x='+(s.root[0]*100).toFixed(1)+' см · y='+(s.root[1]*100).toFixed(1)+' см';
   slipLine.textContent='Скорость контакта (макс.): '+s.slipMmS.toFixed(1)+' мм/с'+(loaded<3?' · МЕНЕЕ 3 ОПОР!':'');
   slipLine.style.color=loaded<3||s.slipMmS>5?'#ffb1a4':'#dce9ed';
   clock.textContent=seconds.toFixed(2)+' / '+run.duration_s.toFixed(2)+' с';slider.value=String(seconds);lastDisplay=seconds;
 }
}
function cameraReset(){const a=current?.root||[0,0,.154];const c=new THREE.Vector3(a[0],a[2],-a[1]);orbit.target.copy(c);const factor=Math.min(2,Math.max(1,1/Math.max(.52,viewer.clientWidth/Math.max(1,viewer.clientHeight))));camera.position.copy(c).add(new THREE.Vector3(.54,.4,.67).multiplyScalar(factor));cameraCenter.copy(c);orbit.update();}
el('reset-camera').addEventListener('click',cameraReset);
function changeRun(id){run=pack?.runs?.[id];if(!run)throw Error('Нет физического опыта '+id);seconds=0;lastDisplay=-1;playing=false;toggle.textContent='▶ Смотреть';slider.min='0';slider.max=String(run.duration_s);slider.value='0';renderFrame(frameAt(run,0));cameraReset();
 note.textContent='Запись mj_step · масса '+run.mass_kg.toFixed(2)+' кг · μ×'+run.friction_scale+' · предел '+run.torque_limit_Nm+' Н·м · мин. опор '+run.summary.min_contacts+'/6 · насыщение '+(run.summary.motor_saturation*100).toFixed(1)+'% · не тест на железе';
}
function populateRuns(){runSelect.replaceChildren();for(const id of Object.keys(pack.runs)){const o=document.createElement('option');o.value=id;o.textContent=LABELS[id]||id;runSelect.append(o);}const prioritized=['stand_yaw_plus15','tripod_forward','tripod_turn','wave_forward','wave_turn'];const wanted=prioritized.find(id=>pack.runs[id]);if(wanted)runSelect.value=wanted;changeRun(runSelect.value);}
async function chooseVariant(v){playing=false;toggle.textContent='▶ Смотреть';variantSelect.disabled=true;runSelect.disabled=true;status.hidden=false;status.textContent='Загружаем кадры MuJoCo…';
 try{if(!cache.has(v))cache.set(v,await json('../data/physics-v12-'+v+'.json'));
  pack=cache.get(v);validatePack(pack);const expectedModel=v==='upper_mass_full'?'5f7b4b35a7f28ce58f6e44b315e1bf700a03b3f4973cbaaa214e3b8354e5a2e2':toeMeta.model_sha256;
  if(pack.variant!==v||pack.original_v12_scene_sha256!==expectedModel)throw Error('Неверная версия физической модели');
  populateRuns();status.hidden=true;
 }catch(e){status.textContent='Ошибка воспроизведения: '+e.message;console.error(e);}
 finally{variantSelect.disabled=false;runSelect.disabled=false;}
}
variantSelect.addEventListener('change',()=>chooseVariant(variantSelect.value));
runSelect.addEventListener('change',()=>changeRun(runSelect.value));
toggle.addEventListener('click',()=>{if(!run)return;if(seconds>=run.duration_s-.0001)seconds=0;playing=!playing;toggle.textContent=playing?'Ⅱ Пауза':'▶ Смотреть';});
slider.addEventListener('input',()=>{if(!run)return;seconds=+slider.value;playing=false;toggle.textContent='▶ Смотреть';renderFrame(frameAt(run,seconds));});
window.addEventListener('blur',()=>{playing=false;toggle.textContent='▶ Смотреть';});
document.addEventListener('visibilitychange',()=>{if(document.hidden){playing=false;toggle.textContent='▶ Смотреть';prev=null;}});
addEventListener('keydown',e=>{if(e.code==='Space'&&document.activeElement?.tagName!=='SELECT'&&!document.activeElement?.matches('input')){e.preventDefault();toggle.click();}});
async function boot(){try{await prepareCAD();ready=true;await chooseVariant(variantSelect.value);if(run)cameraReset();}catch(e){status.textContent='Ошибка запуска: '+e.message;console.error(e);}}
function animate(ts){requestAnimationFrame(animate);if(ready&&run&&prev!==null&&document.visibilityState==='visible'&&playing){const dt=Math.max(0,Math.min(.08,(ts-prev)/1000));seconds=Math.min(run.duration_s,seconds+dt*Number(pace.value));if(seconds>=run.duration_s){playing=false;toggle.textContent='▶ Смотреть';}renderFrame(frameAt(run,seconds));}prev=ts;orbit.update();renderer.render(scene,camera);}
boot();requestAnimationFrame(animate);
