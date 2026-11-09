// Original qpos from MuJoCo; no dynamic state is calculated by the player.
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
const el=id=>document.getElementById(id),area=el("viewer"),seek=el("seek"),
  play=el("play"),speed=el("speed");
const scene=new THREE.Scene();scene.background=new THREE.Color("#142a3a");
const camera=new THREE.PerspectiveCamera(44,1,.008,20);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:"high-performance"});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.7));
renderer.outputColorSpace=THREE.SRGBColorSpace;
area.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.minDistance=.12;orbit.maxDistance=3;
scene.add(new THREE.HemisphereLight(0xe0f1ff,0x3e5262,2.7));
let sun=new THREE.DirectionalLight(0xffffff,2.3);sun.position.set(-.6,1.1,1);scene.add(sun);
let ground=new THREE.Mesh(new THREE.PlaneGeometry(7,7),new THREE.MeshStandardMaterial({color:0x233848,roughness:.9}));
ground.rotation.x=-Math.PI/2;scene.add(ground);
const grid=new THREE.GridHelper(6,60,0x526b78,0x354b57);grid.position.y=.0005;scene.add(grid);
// MuJoCo z-up coordinates are retained inside this transformed group.
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const robot=new THREE.Group();world.add(robot);
const material=(color,metalness=.2)=>new THREE.MeshStandardMaterial({color,metalness,roughness:.51});
const shell=material(0x6fa6c2,.48),steel=material(0xbdcbd5,.65),
  rubber=material(0x181b22,.1),gold=material(0xffb35b,.48);
function shape(parent,geometry,mat,x=0,y=0,z=0){
  const obj=new THREE.Mesh(geometry,mat);obj.position.set(x,y,z);parent.add(obj);return obj;
}
shape(robot,new THREE.BoxGeometry(.094,.068,.048),shell);
shape(robot,new THREE.BoxGeometry(.068,.054,.002),steel,0,0,.025);
function leg(side){
  const hip=new THREE.Group();hip.position.set(0,side*.043,-.016);robot.add(hip);
  const diff=new THREE.Vector3(.014,0,-.05);
  let link=shape(hip,new THREE.CylinderGeometry(.007,.007,diff.length(),14),steel,.007,0,-.025);
  link.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),diff.clone().normalize());
  shape(hip,new THREE.SphereGeometry(.01,12,10),gold);
  const wheel=new THREE.Group();wheel.position.set(.014,0,-.05);hip.add(wheel);
  shape(wheel,new THREE.CylinderGeometry(.024,.024,.018,32),rubber);
  shape(wheel,new THREE.CylinderGeometry(.009,.009,.020,24),gold);
  for(let i=0;i<6;i++){let s=shape(wheel,new THREE.BoxGeometry(.035,.02,.002),steel);s.rotation.y=i*Math.PI/3;}
  return {hip,wheel};
}
const left=leg(1),right=leg(-1);
let data=null,frames=[],t=0,running=true,last=null,follow=new THREE.Vector3();
const q1=new THREE.Quaternion(),q2=new THREE.Quaternion();
function resize(){let w=Math.max(area.clientWidth,1),h=Math.max(area.clientHeight,1);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener("resize",resize);resize();
function at(s){
  if(s<=frames[0][0])return {a:frames[0],b:frames[0],k:0};
  if(s>=frames[frames.length-1][0])return {a:frames[frames.length-1],b:frames[frames.length-1],k:0};
  let lo=0,hi=frames.length-1;
  while(hi-lo>1){let m=(lo+hi)>>1;if(frames[m][0]<=s)lo=m;else hi=m;}
  return {a:frames[lo],b:frames[hi],k:(s-frames[lo][0])/(frames[hi][0]-frames[lo][0])};
}
function draw(s){
  if(!data)return;
  const {a,b,k}=at(s),v=i=>a[i]+(b[i]-a[i])*k;
  robot.position.set(v(1),v(2),v(3));
  q1.set(a[5],a[6],a[7],a[4]);q2.set(b[5],b[6],b[7],b[4]);robot.quaternion.copy(q1).slerp(q2,k);
  left.hip.rotation.y=v(8);left.wheel.rotation.y=v(9);
  right.hip.rotation.y=v(10);right.wheel.rotation.y=v(11);
  const target=new THREE.Vector3(v(1),v(3),-v(2)),delta=target.clone().sub(follow);
  camera.position.add(delta);orbit.target.add(delta);follow.copy(target);
  const secs=n=>Math.floor(n/60)+":"+String(Math.floor(n%60)).padStart(2,"0");
  el("time").textContent=secs(s)+" / "+secs(data.duration_s);
  el("pose").textContent="Наклон "+(v(15)*180/Math.PI).toFixed(1)+"°, x="+v(1).toFixed(3)+" м";
  el("joints").textContent="Ноги "+(v(8)*180/Math.PI).toFixed(0)+"° / "+(v(10)*180/Math.PI).toFixed(0)+"° · 4 привода";
  el("contacts").textContent="Опора "+(v(12)>.5?"есть":"нет")+(v(13)>.5?", корпус на полу":"")+", F="+Math.max(0,v(14)).toFixed(2)+" Н";
  seek.value=String(s);
}
function resetCamera(){if(!data)return;let a=at(t).a,p=new THREE.Vector3(a[1],a[3],-a[2]);orbit.target.copy(p);camera.position.copy(p).add(new THREE.Vector3(.27,.20,.32));follow.copy(p);orbit.update();}
function playState(next){running=next;play.textContent=running?"Ⅱ":"▶";}
play.addEventListener("click",()=>playState(!running));
seek.addEventListener("input",()=>{t=Number(seek.value);draw(t);});
el("reset-camera").addEventListener("click",resetCamera);
async function init(){
  try{
    const response=await fetch("/hexapod/data/navbot-en01-template.json",{cache:"no-store"});
    if(!response.ok)throw Error("HTTP "+response.status);
    data=await response.json();
    if(data.format!=="navbot-en01-mujoco-physical-replay-v1"||data.frame_count!==data.frames?.length||
       data.sample_stride!==16||data.frame_count<50||!data.source?.trace_sha256||
       !["research","failed"].includes(data.status))throw Error("Непроверенный формат");
    let last=-1;
    for(const f of data.frames){if(!Array.isArray(f)||f.length!==16||!f.every(Number.isFinite)||f[0]<=last)throw Error("Повреждена физическая трасса");last=f[0];}
    frames=data.frames;seek.max=String(data.duration_s);
    el("result").textContent=data.status==="failed"?"FAIL — прыжок не получился":"Research — предварительно";
    el("result").style.color=data.status==="failed"?"#ffb5a5":"#f3dcaa";
    el("description").textContent=data.detail;
    draw(0);resetCamera();
  }catch(e){let m=el("replay-error");m.hidden=false;m.textContent="Ошибка загрузки MuJoCo: "+String(e.message||e);playState(false);}
}
function tick(now){requestAnimationFrame(tick);if(running&&data&&last!==null){t+=Math.max(0,Math.min(.1,(now-last)/1000))*Number(speed.value||1);if(t>data.duration_s)t=0;draw(t);}last=now;orbit.update();renderer.render(scene,camera);}
requestAnimationFrame(tick);init();
