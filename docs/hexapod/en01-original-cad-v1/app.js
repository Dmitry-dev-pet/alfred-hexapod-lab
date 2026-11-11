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
let originalCAD;
function decoded(value,Type){const bytes=Uint8Array.from(atob(value),c=>c.charCodeAt(0));if(bytes.length%4)throw Error('Повреждена геометрия');return new Type(bytes.buffer);}
function robot(){
  root=new THREE.Group();zUp.add(root);
  function side(h,w){const hip=new THREE.Group();hip.position.fromArray(h);root.add(hip);const wheel=new THREE.Group();wheel.position.fromArray(w).sub(hip.position);hip.add(wheel);return {hip,wheel};}
  const r=originalCAD.rig;left=side(r.left_hip,r.left_wheel);right=side(r.right_hip,r.right_wheel);
  for(const part of originalCAD.parts){
    const vertices=decoded(part.vertices_f32le,Float32Array),indices=decoded(part.indices_u32le,Uint32Array);
    if(vertices.length!==part.vertex_count*3||indices.length!==part.triangles*3||!vertices.every(Number.isFinite)||indices.some(i=>i>=part.vertex_count))throw Error('Повреждена деталь CAD');
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(vertices,3));geometry.setIndex(new THREE.BufferAttribute(indices,1));geometry.computeVertexNormals();
    let parent=root,offset=new THREE.Vector3();
    if(part.group==='left_leg'){parent=left.hip;offset.fromArray(r.left_hip);}
    else if(part.group==='right_leg'){parent=right.hip;offset.fromArray(r.right_hip);}
    else if(part.group==='left_wheel'){parent=left.wheel;offset.fromArray(r.left_wheel);}
    else if(part.group==='right_wheel'){parent=right.wheel;offset.fromArray(r.right_wheel);}
    else if(part.group!=='body')throw Error('Неизвестная группа детали');
    geometry.translate(-offset.x,-offset.y,-offset.z);
    const color=new THREE.Color().setRGB(part.color[0],part.color[1],part.color[2],THREE.SRGBColorSpace);
    const material=new THREE.MeshStandardMaterial({color,roughness:.48,metalness:.2,side:THREE.DoubleSide});
    const mesh=new THREE.Mesh(geometry,material);mesh.name=part.name;parent.add(mesh);
  }
}
async function loadCAD(){
  const response=await fetch('./original-meshes.json',{cache:'no-store'});if(!response.ok)throw Error('CAD HTTP '+response.status);
  originalCAD=await response.json();
  if(originalCAD.schema!=='en01-original-step-mesh/v1'||originalCAD.source.git_blob_sha1!=='9156683a0bb694c84dc541a3e26b5f6b08db1a41'||originalCAD.parts?.length!==originalCAD.part_count||originalCAD.part_count!==73)throw Error('Неверный исходный CAD');
  robot();$('cad-loading').hidden=true;document.documentElement.dataset.cadParts=String(originalCAD.part_count);
  $('cad-source').textContent='73 детали · геометрия из оригинального STEP';$('model').disabled=false;$('case').disabled=false;
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
    const response=await fetch('../en01-experiments-v1/'+version+'-'+mode+'.json',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);
    const next=await response.json();if(id!==loadId)return;validate(next);data=next;frames=data.frames;
    $('verdict').textContent=data.status==='failed'?'FAIL · прыжок не получился':'Исследование · удержание с дрейфом';
    $('verdict').style.color=data.status==='failed'?'#ffae99':'#e9d68b';
    $('metrics').textContent='Дрейф до '+(data.result.max_abs_x_m*100).toFixed(1)+' см · наклон до '+data.result.max_abs_pitch_deg.toFixed(1)+'° · 6 с';
    $('limitations').textContent='73 детали из оригинального STEP. '+data.model_label+'. CAD — визуальная оболочка; контакты и силы относятся к прежней приближённой модели. Детали ног следуют записанным углам как жёсткие группы; четырёхзвенная кинематика не восстановлена.';
    $('provenance').textContent='Исходная CAD-сборка: 3139f2fb… · 73 детали · MuJoCo 3.14.0 · 300 сохранённых кадров · шаг 0,002 с · SHA-256 исходной трассы: '+data.source.trace_sha256;
    draw();resetCamera();run(true);document.documentElement.dataset.replay=version+'-'+mode;
  }catch(e){if(id!==loadId)return;$('verdict').textContent='Запись не загружена';error('Не удалось загрузить запись: '+e.message);}
}
$('model').addEventListener('change',load);$('case').addEventListener('change',load);
play.addEventListener('click',()=>run(!running));seek.addEventListener('input',()=>{time=Number(seek.value);draw();});$('reset-camera').addEventListener('click',resetCamera);
function tick(now){requestAnimationFrame(tick);if(data&&running&&previous!==null){time+=Math.min(.1,(now-previous)/1000)*Number($('speed').value);if(time>6)time=0;draw();}previous=now;if(renderer){controls.update();renderer.render(scene,camera);}}
requestAnimationFrame(tick);loadCAD().then(load).catch(e=>{error('Не удалось загрузить оригинальную геометрию: '+e.message);$('cad-loading').hidden=true;run(false);});
