import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id);
const area=$('viewer'),seek=$('seek'),play=$('play');
let renderer,controls,root,left,right,data,frames=[],time=0,running=true,previous=null,loadId=0;
const scene=new THREE.Scene();scene.background=new THREE.Color('#122836');
const camera=new THREE.PerspectiveCamera(46,1,.008,20);
const follow=new THREE.Vector3(),q1=new THREE.Quaternion(),q2=new THREE.Quaternion();
function error(message){$('replay-error').hidden=false;$('replay-error').textContent=message;}
try{
  renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.7));renderer.outputColorSpace=THREE.SRGBColorSpace;
  area.appendChild(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);
  controls.enableDamping=true;controls.minDistance=.12;controls.maxDistance=4;
}catch(e){error('3D недоступно: браузер не создал WebGL-контекст. Данные записи и выбор сценария доступны.');}
scene.add(new THREE.HemisphereLight(0xe0f2ff,0x394d55,2.6));
const light=new THREE.DirectionalLight(0xffffff,2.3);light.position.set(-.65,1.4,.85);scene.add(light);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(6,6),new THREE.MeshStandardMaterial({color:0x243c4c,roughness:.97}));ground.rotation.x=-Math.PI/2;scene.add(ground);
const grid=new THREE.GridHelper(6,60,0x5a7682,0x38505e);grid.position.y=.0005;scene.add(grid);
const zUp=new THREE.Group();zUp.rotation.x=-Math.PI/2;scene.add(zUp);
const materials=[0x68a2bf,0xbdcbd3,0x161b23,0xf9b55f].map(color=>new THREE.MeshStandardMaterial({color,roughness:.5,metalness:.25}));
function mesh(parent,geometry,material,pos=[0,0,0]){const m=new THREE.Mesh(geometry,material);m.position.fromArray(pos);parent.add(m);return m;}
function robot(g){
  if(root){zUp.remove(root);root.traverse(o=>o.geometry?.dispose());}
  root=new THREE.Group();zUp.add(root);
  mesh(root,new THREE.BoxGeometry(...g.torso_size.map(v=>v*2)),materials[0]);
  function side(hipPos,wheelPos){
    const hip=new THREE.Group();hip.position.fromArray(hipPos);root.add(hip);
    const v=new THREE.Vector3(...wheelPos);
    const bone=mesh(hip,new THREE.CylinderGeometry(g.leg_radius,g.leg_radius,v.length(),16),materials[1],v.clone().multiplyScalar(.5).toArray());
    bone.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),v.clone().normalize());
    mesh(hip,new THREE.SphereGeometry(.009,16,12),materials[3]);
    const wheel=new THREE.Group();wheel.position.fromArray(wheelPos);hip.add(wheel);
    mesh(wheel,new THREE.CylinderGeometry(g.wheel_radius,g.wheel_radius,g.wheel_width,40),materials[2]);
    mesh(wheel,new THREE.CylinderGeometry(g.wheel_radius*.4,g.wheel_radius*.4,g.wheel_width+.002,24),materials[3]);
    for(let a=0;a<6;a++){const spoke=mesh(wheel,new THREE.BoxGeometry(g.wheel_radius*1.45,g.wheel_width+.003,.002),materials[1]);spoke.rotation.y=a*Math.PI/3;}
    return {hip,wheel};
  }
  left=side(g.left_hip,g.left_wheel);right=side(g.right_hip,g.right_wheel);
}
function resize(){if(!renderer)return;const w=Math.max(1,area.clientWidth),h=Math.max(1,area.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function frameAt(s){
  if(s<=frames[0][0])return {a:frames[0],b:frames[0],k:0};
  if(s>=frames.at(-1)[0])return {a:frames.at(-1),b:frames.at(-1),k:0};
  let lo=0,hi=frames.length-1;while(hi-lo>1){const mid=(lo+hi)>>1;if(frames[mid][0]<=s)lo=mid;else hi=mid;}
  return {a:frames[lo],b:frames[hi],k:(s-frames[lo][0])/(frames[hi][0]-frames[lo][0])};
}
function draw(){
  if(!data)return;const {a,b,k}=frameAt(time),lerp=i=>a[i]+(b[i]-a[i])*k;
  root.position.set(lerp(1),lerp(2),lerp(3));q1.set(a[5],a[6],a[7],a[4]);q2.set(b[5],b[6],b[7],b[4]);root.quaternion.copy(q1).slerp(q2,k);
  left.hip.rotation.y=lerp(8);left.wheel.rotation.y=lerp(9);right.hip.rotation.y=lerp(10);right.wheel.rotation.y=lerp(11);
  const target=new THREE.Vector3(lerp(1),lerp(3),-lerp(2));const delta=target.clone().sub(follow);
  if(controls){camera.position.add(delta);controls.target.add(delta);}follow.copy(target);
  $('time').textContent=time.toFixed(1)+' / 6,0 с';seek.value=String(time);
  $('pose').textContent='Наклон '+(lerp(15)*180/Math.PI).toFixed(1)+'° · x '+lerp(1).toFixed(3)+' м';
  $('contact').textContent=(a[12]?'Опора есть':'Нет опоры')+(a[13]?' · корпус на полу':'')+' · '+a[14].toFixed(2)+' Н';
}
function resetCamera(){if(!data||!controls)return;const {a}=frameAt(time);const target=new THREE.Vector3(a[1],a[3],-a[2]);controls.target.copy(target);camera.position.copy(target).add(new THREE.Vector3(.25,.20,.33));follow.copy(target);controls.update();}
function run(next){running=next;play.textContent=next?'Ⅱ':'▶';play.setAttribute('aria-label',next?'Пауза':'Воспроизвести');}
function validate(d){
  if(d.format!=='navbot-en01-mujoco-physical-replay-v1'||d.frame_count!==300||d.frames?.length!==300||d.sample_stride!==16||!['research','failed'].includes(d.status)||!d.geometry)throw Error('Неверный формат записи');
  let last=-1;for(const f of d.frames){if(f.length!==16||!f.every(Number.isFinite)||f[0]<=last)throw Error('Повреждён кадр');last=f[0];}
}
async function load(){
  const id=++loadId;data=null;frames=[];time=0;previous=null;run(false);$('verdict').textContent='Загружаем запись…';
  const version=$('model').value,mode=$('case').value;
  try{
    const response=await fetch('./'+version+'-'+mode+'.json',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);
    const next=await response.json();if(id!==loadId)return;validate(next);data=next;frames=data.frames;robot(data.geometry);
    $('verdict').textContent=data.status==='failed'?'FAIL · прыжок не получился':'Исследование · удержание с дрейфом';
    $('verdict').style.color=data.status==='failed'?'#ffae99':'#e9d68b';
    $('metrics').textContent='Дрейф до '+(data.result.max_abs_x_m*100).toFixed(1)+' см · наклон до '+data.result.max_abs_pitch_deg.toFixed(1)+'° · 6 с';
    $('limitations').textContent=data.model_label+'. Четырёхзвенный механизм, массы и параметры приводов не проверены. Успешных прыжков нет.';
    $('provenance').textContent='MuJoCo 3.14.0 · 300 сохранённых кадров · шаг 0,002 с · SHA-256 исходной трассы: '+data.source.trace_sha256;
    draw();resetCamera();run(true);document.documentElement.dataset.replay=version+'-'+mode;
  }catch(e){if(id!==loadId)return;$('verdict').textContent='Запись не загружена';error('Не удалось загрузить запись: '+e.message);}
}
$('model').addEventListener('change',load);$('case').addEventListener('change',load);
play.addEventListener('click',()=>run(!running));seek.addEventListener('input',()=>{time=Number(seek.value);draw();});$('reset-camera').addEventListener('click',resetCamera);
function tick(now){requestAnimationFrame(tick);if(data&&running&&previous!==null){time+=Math.min(.1,(now-previous)/1000)*Number($('speed').value);if(time>6)time=0;draw();}previous=now;if(renderer){controls.update();renderer.render(scene,camera);}}
requestAnimationFrame(tick);load();
