import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// This module only REPLAYS measured MuJoCo 3.14 states from the original simulator.
// It does not compute dynamics, contacts or controller outputs in the browser.
const q = s => document.querySelector(s);
const viewer=q('#viewer'), play=q('#play'), seek=q('#seek'), speed=q('#speed');
const error=q('#rider-error'), selector=q('#rider-case'), stat=q('#rider-status');
const phase=q('#rider-phase'), measure=q('#rider-measure'), loads=q('#rider-loads');
const clock=q('#clock'), endClock=q('#end-clock'), follow=q('#follow');
const contacts=q('#contacts'), pathToggle=q('#path');
const FILES={
  rl:'/hexapod/data/rider2-rl-drive-12s.json',
  linear:'/hexapod/data/rider2-linear-drive-12s.json',
  corner:'/hexapod/data/rider2-rl-corner-fail.json'
};
const TITLES={rl:'RL · движение и толчки',linear:'Linear · движение и толчки',corner:'RL · экстремальное падение'};
const scene=new THREE.Scene();scene.background=new THREE.Color('#101e2a');
scene.fog=new THREE.Fog('#101e2a',2.5,8);
const camera=new THREE.PerspectiveCamera(43,1,.01,20);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.7));
renderer.outputColorSpace=THREE.SRGBColorSpace;viewer.append(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.07;orbit.minDistance=.16;orbit.maxDistance=4;orbit.maxPolarAngle=Math.PI*.495;
scene.add(new THREE.HemisphereLight(0xc9eaff,0x344b58,2.4));
const light=new THREE.DirectionalLight(0xffffff,2.5);light.position.set(-1.7,4,2.4);scene.add(light);
const fill=new THREE.DirectionalLight(0x93d4f2,1);fill.position.set(1,1,-1.5);scene.add(fill);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(25,25),new THREE.MeshStandardMaterial({color:0x162735,roughness:.91}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.003;scene.add(floor);
const grid=new THREE.GridHelper(8,80,0x476779,0x294356);grid.position.y=.001;scene.add(grid);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world); // MuJoCo Z-up -> Three Y-up
const body=new THREE.Group();world.add(body);
const material=(color,metalness=.3,roughness=.52)=>new THREE.MeshStandardMaterial({color,metalness,roughness});
const shell=material(0x30485d,.45),panel=material(0xdfedf0,.17),dark=material(0x161e28,.2),rim=material(0x6dc4ca,.5),accent=material(0x86e4bc,.26);
const add=(group,geometry,mat,pos)=>{const mesh=new THREE.Mesh(geometry,mat);if(pos)mesh.position.set(...pos);group.add(mesh);return mesh;};
add(body,new THREE.BoxGeometry(.09,.072,.05),shell,[0,0,0]);
add(body,new THREE.BoxGeometry(.072,.058,.004),panel,[0,0,.027]);
add(body,new THREE.BoxGeometry(.004,.057,.026),accent,[.047,0,.004]);
add(body,new THREE.BoxGeometry(.044,.009,.010),dark,[-.017,0,.034]);
const wheels=[];
for(const [side,y,idx] of [['left',.0433,0],['right',-.0433,1]]){
  const stem=new THREE.Group();body.add(stem);stem.position.set(0,y,-.081);
  add(stem,new THREE.CylinderGeometry(.0045,.0045,.051,10),rim,[0,0,.021]);
  const wheel=new THREE.Group();stem.add(wheel);
  add(wheel,new THREE.CylinderGeometry(.0308,.0308,.0157,36),dark);
  add(wheel,new THREE.CylinderGeometry(.019,.019,.0163,32),rim);
  add(wheel,new THREE.CylinderGeometry(.008,.008,.017,28),panel);
  for(let k=0;k<3;k++){
    const bar=add(wheel,new THREE.BoxGeometry(.043,.0181,.0042),panel);bar.rotation.y=k*Math.PI/3;
  }
  // Contact ring is in world coordinates; its visibility comes from actual Fz.
  const ring=add(world,new THREE.TorusGeometry(.037,.0027,6,30),accent);ring.position.z=.003;
  wheels.push({stem,wheel,ring,idx,side});
}
const qA=new THREE.Quaternion(),qB=new THREE.Quaternion(),qNow=new THREE.Quaternion();
const cA=new THREE.Vector3(),cB=new THREE.Vector3(),worldWheel=new THREE.Vector3();
let current=null,time=0,running=true,last=null,trail=null,previousRoot=new THREE.Vector3();
let req=0;
const fmt=t=>`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function check(data){
  if(!data||data.format!=='xgorider2-mujoco-physical-replay-v1'||data.robot!=='XGO-Rider2')throw Error('Неверный формат Rider2');
  if(!Array.isArray(data.frames)||data.frames.length<2||data.frame_count!==data.frames.length||data.sample_stride!==36)throw Error('Неверное число кадров');
  for(let i=0;i<data.frames.length;i++){
    const f=data.frames[i];if(!Array.isArray(f)||f.length!==36||!f.every(Number.isFinite)||i>0&&!(f[0]>data.frames[i-1][0]))throw Error('Некорректные физические кадры');
  }
  if(Math.abs(data.frames[data.frames.length-1][0]-data.duration_s)>.0001)throw Error('Длительность не соответствует трассе');
  if(data.joint_qpos_indices.extension12!==7||data.joint_qpos_indices.wheel11!==8||data.joint_qpos_indices.extension22!==9||data.joint_qpos_indices.wheel21!==10)throw Error('Неверный порядок суставов');
}
function at(t){const f=current.frames;let lo=0,hi=f.length-1;while(hi-lo>1){const mid=(lo+hi)>>1;if(f[mid][0]<t)lo=mid;else hi=mid;}const a=f[lo],b=f[hi];const k=Math.max(0,Math.min(1,(t-a[0])/(b[0]-a[0]||1)));return {a,b,k,nearest:k<.5?a:b};}
const lerp=(a,b,idx,k)=>a[idx]+(b[idx]-a[idx])*k;
function cameraReset(){if(!current)return;const row=at(time).nearest;const root=new THREE.Vector3(row[1],row[3],-row[2]);orbit.target.copy(root);const ratio=viewer.clientWidth/Math.max(1,viewer.clientHeight);const z=ratio<.9?Math.min(1.8,.9/ratio):1;camera.position.copy(root).add(new THREE.Vector3(.30,.24,.45).multiplyScalar(z));previousRoot.copy(root);orbit.update();}
function renderFrame(t){if(!current)return;
  const {a,b,k,nearest}=at(t);
  const root=new THREE.Vector3(lerp(a,b,1,k),lerp(a,b,3,k),-lerp(a,b,2,k));
  cA.copy(root);
  if(follow.checked){const delta=root.clone().sub(previousRoot);camera.position.add(delta);orbit.target.add(delta);}previousRoot.copy(root);
  body.position.set(lerp(a,b,1,k),lerp(a,b,2,k),lerp(a,b,3,k));
  qA.set(a[5],a[6],a[7],a[4]);qB.set(b[5],b[6],b[7],b[4]);qNow.copy(qA).slerp(qB,k);
  body.quaternion.copy(qNow);
  wheels.forEach((w,i)=>{
    w.stem.position.z=-.081-lerp(a,b,i===0?8:10,k);
    w.wheel.rotation.y=lerp(a,b,i===0?9:11,k);
    // Exact wheel-center kinematic transform from the published Rider2 MJCF.
    worldWheel.set(0,i===0?.0433:-.0433,-.081-lerp(a,b,i===0?8:10,k));
    worldWheel.applyQuaternion(qNow).add(body.position);
    w.ring.position.set(worldWheel.x,worldWheel.y,.003);
    w.ring.visible=contacts.checked&&nearest[22+i]>.2;
  });
  const p=nearest[27]*180/Math.PI,roll=nearest[26]*180/Math.PI;
  const stopped=current.terminated&&t>=current.duration_s-.005;
  const label=stopped?'ПАДЕНИЕ · критерий MuJoCo':current.algorithm==='native_trained_rl'?'Оригинальная обученная RL':'Линейный контроллер';
  phase.textContent=label;
  phase.className=stopped?'rider-fail':current.algorithm==='native_trained_rl'?'rider-rl':'rider-linear';
  measure.textContent=`Наклон ${p.toFixed(1)}° · крен ${roll.toFixed(1)}° · скорость ${nearest[29].toFixed(2)} м/с`;
  const push=nearest[35]===1?' · ТОЛЧОК':'';
  loads.textContent=`Опоры: ${nearest[22].toFixed(1)} Н / ${nearest[23].toFixed(1)} Н · цель ${nearest[30].toFixed(2)} м/с${push}`;
  loads.classList.toggle('push',nearest[35]===1);
  stat.textContent=`${current.algorithm==='native_trained_rl'?'RL v6':'Linear'} · MuJoCo ${current.physics_version} · seed ${current.seed} · ${current.frame_count} измеренных кадров`;
  seek.value=String(t);seek.style.setProperty('--pct',`${100*t/current.duration_s}%`);clock.textContent=fmt(t);
}
function setPlaying(b){running=b;play.textContent=running?'Ⅱ':'▶';play.setAttribute('aria-label',running?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>setPlaying(!running));
seek.addEventListener('input',()=>{time=Number(seek.value);renderFrame(time);if(current?.terminated&&time>=current.duration_s-.005)setPlaying(false);});
contacts.addEventListener('change',()=>renderFrame(time));
pathToggle.addEventListener('change',()=>{if(trail)trail.visible=pathToggle.checked;});
q('#reset-camera').addEventListener('click',cameraReset);
window.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT','BUTTON'].includes(document.activeElement?.tagName)){e.preventDefault();setPlaying(!running);}});
async function load(name){const serial=++req;error.hidden=true;const wasRunning=running;setPlaying(false);stat.textContent='Загружаем физическую трассу…';
  try{const res=await fetch(FILES[name],{cache:'no-store'});if(!res.ok)throw Error(`HTTP ${res.status}`);const obj=await res.json();check(obj);if(serial!==req)return;
    current=obj;time=0;seek.max=String(obj.duration_s);endClock.textContent=fmt(obj.duration_s);
    if(trail){scene.remove(trail);trail.geometry.dispose();trail.material.dispose();trail=null;}
    const pts=obj.frames.map(f=>new THREE.Vector3(f[1],.009,-f[2]));
    trail=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:0x5ad6b4,opacity:.65,transparent:true}));scene.add(trail);trail.visible=pathToggle.checked;
    previousRoot.set(obj.frames[0][1],obj.frames[0][3],-obj.frames[0][2]);renderFrame(0);cameraReset();
    setPlaying(wasRunning||serial===1);
  }catch(e){if(serial!==req)return;error.hidden=false;error.textContent='Не удалось открыть физическую запись: '+e.message;setPlaying(false);}
}
selector.addEventListener('change',()=>load(selector.value));
function tick(ts){requestAnimationFrame(tick);if(last!==null&&running&&current){const next=time+Math.min(.1,(ts-last)/1000)*Number(speed.value);if(next>=current.duration_s){if(current.terminated){time=current.duration_s;setPlaying(false);}else time=next%current.duration_s;}else time=next;renderFrame(time);}last=ts;orbit.update();renderer.render(scene,camera);}
requestAnimationFrame(tick);load(selector.value);
