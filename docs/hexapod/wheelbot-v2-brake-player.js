import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

// Faithful replay of recorded MuJoCo mj_step qpos; not live simulation.
// Original CAD is fetched ONLY from immutable upstream URLs, SHA256 checked,
// then cached by the browser for repeat/offline playback. No mesh redistribution.
const upstream='https://raw.githubusercontent.com/wheelbot/Mini-Wheelbot/';
const upstreamCommit='73815e49371c5fb5f49eec59d364e4d9d1b57b10';
const meshes=[
  {name:'body.stl',size:12754984,sha:'4154eec8449c616db2da265930e81633bb8daaa87530f809f2de247f9e66607d'},
  {name:'actuator_rotor.stl',size:1035784,sha:'d5c9b8392ae6d88aa037b64700ea7e274ca4f8ff3e1873556567f6173f2b92bf'}
];
const $=id=>document.getElementById(id);
const caseId=document.querySelector('.layout')?.dataset?.caseId;
const state=$('state'), stats=$('pose'), motors=$('motors'), warn=$('error');
const rootArea=$('viewer'), nativePlay=$('play'), nativeSeek=$('seek'), nativeSpeed=$('speed');
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(1.5,window.devicePixelRatio||1));
renderer.outputColorSpace=THREE.SRGBColorSpace;
rootArea.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color('#10212c');
const camera=new THREE.PerspectiveCamera(43,1,0.005,20);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.minDistance=.13;orbit.maxDistance=2.0;
scene.add(new THREE.HemisphereLight(0xdbebfc,0x36516a,3.0));
const key=new THREE.DirectionalLight(0xffffff,3);key.position.set(-.7,1,1);scene.add(key);
const fill=new THREE.DirectionalLight(0xc7ebff,1);fill.position.set(.8,.6,-.5);scene.add(fill);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(8,8),new THREE.MeshStandardMaterial({color:0x233b47,roughness:.9}));
ground.rotation.x=-Math.PI/2;scene.add(ground);
scene.add(new THREE.GridHelper(5,60,0x64859a,0x344f62));
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const base=new THREE.Group();world.add(base);
const drive=new THREE.Group();drive.position.z=-.0325;base.add(drive);
const reaction=new THREE.Group();reaction.position.z=.0325;base.add(reaction);
const originalMetal=new THREE.MeshStandardMaterial({color:0xa6c1d3,roughness:.42,metalness:.48,side:THREE.DoubleSide});
const originalWheel=new THREE.MeshStandardMaterial({color:0xe6ba62,roughness:.44,metalness:.58,side:THREE.DoubleSide});

let dataset=null, frames=[],time=0,lastAnimation=null,running=true,previousWorldPos=new THREE.Vector3();
const q1=new THREE.Quaternion(),q2=new THREE.Quaternion();

function setError(msg){
  warn.hidden=false;warn.textContent=msg;state.textContent='Не удалось загрузить запись';
  running=false;nativePlay.textContent='▶';nativePlay.setAttribute('aria-label','Воспроизвести');
}
function resize(){
  const w=Math.max(rootArea.clientWidth,1),h=Math.max(rootArea.clientHeight,1);
  camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);
}
window.addEventListener('resize',resize);resize();

async function sha256(buffer){
  if(!crypto?.subtle)throw Error('В браузере недоступна проверка SHA-256');
  const digest=await crypto.subtle.digest('SHA-256',buffer);
  return [...new Uint8Array(digest)].map(x=>x.toString(16).padStart(2,'0')).join('');
}
async function getOriginalSTL(file){
  const url=upstream+upstreamCommit+'/hardware/mechanical/meshes/'+file.name;
  const cache='caches' in window?await caches.open('hexapod-wheelbot-official-cad-v1'):null;
  let saved=cache?await cache.match(url):null, bytes, fromCache=false;
  if(saved){bytes=await saved.arrayBuffer();fromCache=true;}
  else{
    state.textContent='Загружаем оригинальный CAD '+file.name+' ('+(file.size/1e6).toFixed(1)+' МБ)…';
    const r=await fetch(url,{mode:'cors'});
    if(!r.ok)throw Error(file.name+': HTTP '+r.status+'. Нужен интернет для первого просмотра.');
    bytes=await r.arrayBuffer();
  }
  const got=await sha256(bytes);
  if(got!==file.sha || bytes.byteLength!==file.size){
    if(cache)await cache.delete(url);
    throw Error('Неверный SHA-256 или размер CAD '+file.name);
  }
  if(!fromCache && cache){
    await cache.put(url,new Response(bytes.slice(0),{headers:{'Content-Type':'application/octet-stream'}}));
  }
  return new STLLoader().parse(bytes);
}
async function loadReplay(){
  const res=await fetch('/hexapod/data/'+caseId+'.json',{cache:'no-store'});
  if(!res.ok)throw Error('Отсутствуют записанные кадры '+caseId+' ('+res.status+')');
  const pack=await res.json();
  if(pack.format!=='wheelbot-v2-gzip-b64-frames-v1' ||
     !Number.isInteger(pack.frame_count) || pack.frame_count<150 ||
     typeof pack.base64!=='string' || pack.compression!=='gzip' ||
     !('DecompressionStream' in window))throw Error('Пакет кадров не соответствует протоколу (требуется современный Chrome)');
  const compressed=Uint8Array.from(atob(pack.base64),x=>x.charCodeAt(0));
  const stream=new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'));
  const bytes=await new Response(stream).arrayBuffer();
  if(bytes.byteLength!==pack.raw_bytes || await sha256(bytes)!==pack.raw_sha256)
    throw Error('SHA-256 траектории не совпадает');
  const result=JSON.parse(new TextDecoder().decode(bytes));
  if(result.format!=='wheelbot-v2-physical-mujoco-state-v1' ||
     result.case_id!==caseId || result.status!=='failed' ||
     result.source_trace_sha256!==pack.source_trace_sha256 ||
     result.engine!=='MuJoCo 3.14.0' || result.source_commit!==upstreamCommit ||
     !Array.isArray(result.frames)||result.frames.length!==pack.frame_count ||
     result.summary.numerical_warning !== null ||
     result.summary.speed_cap_assumed!==true)
    throw Error('Неверный источник или завершение физического эксперимента');
  let last=-1;
  for(const f of result.frames){
    if(!Array.isArray(f)||f.length!==14 ||!f.every(Number.isFinite)||f[0]<=last)
      throw Error('Повреждённый или несортированный кадр');
    last=f[0];
  }
  return result;
}
async function makeRobot(){
  const [bodyGeom,rotorGeom]=await Promise.all(meshes.map(getOriginalSTL));
  // MuJoCo model maps original CAD Y-up to MuJoCo Z-up by +90° about CAD X.
  for(const geom of [bodyGeom,rotorGeom])geom.scale(.001,.001,.001);
  const bodyMesh=new THREE.Mesh(bodyGeom,originalMetal);
  bodyMesh.rotation.x=Math.PI/2;base.add(bodyMesh);
  // The two *original* rotor CAD meshes move at the actual recorded joint angles.
  // Their placements (±32.5mm) follow source STEP motor seats; identity is inferred.
  const lowerMesh=new THREE.Mesh(rotorGeom,originalWheel);drive.add(lowerMesh);
  const upperMesh=new THREE.Mesh(rotorGeom,originalWheel);
  upperMesh.rotation.z=-Math.PI/2;reaction.add(upperMesh);
}
function interpolate(t){
  let lo=0,hi=frames.length-1;
  if(t<=frames[0][0])return {a:frames[0],b:frames[0],u:0};
  if(t>=frames[hi][0])return {a:frames[hi],b:frames[hi],u:0};
  while(hi-lo>1){const mid=(lo+hi)>>1;if(frames[mid][0]<=t)lo=mid;else hi=mid;}
  const a=frames[lo],b=frames[hi],u=Math.min(1,Math.max(0,(t-a[0])/(b[0]-a[0])));
  return {a,b,u};
}
function draw(t){
  if(!dataset)return;
  const {a,b,u}=interpolate(t),lerp=i=>a[i]+(b[i]-a[i])*u;
  base.position.set(lerp(1),lerp(2),lerp(3));
  q1.set(a[5],a[6],a[7],a[4]);q2.set(b[5],b[6],b[7],b[4]);
  base.quaternion.copy(q1.slerp(q2,u));
  drive.rotation.x=lerp(8);reaction.rotation.y=-lerp(9);
  const mapped=new THREE.Vector3(lerp(1),lerp(3),-lerp(2));
  const shift=mapped.clone().sub(previousWorldPos);
  camera.position.add(shift);orbit.target.add(shift);previousWorldPos.copy(mapped);
  const stage=t<1?'Начало опыта':
              t<4.6?'Разгон маховика':
              t<4.9?'Обратный импульс · 0,30 с':
              caseId.includes('feedback') && t>=5.0?'Ранний экспериментальный LQR':
              'Проверка свободного движения после импульса';
  state.textContent=stage+' · НЕ ВСТАЛ';
  stats.textContent='Наклон '+lerp(10).toFixed(1)+'° · высота '+(lerp(3)*100).toFixed(1)+' см · '+t.toFixed(2)+' / 8,00 с';
  motors.textContent='Маховик '+lerp(11).toFixed(0)+' рад/с · момент '+lerp(12).toFixed(3)+' Н·м · контакты '+Math.round(lerp(13));
  nativeSeek.value=String(t);
}
function resetCamera(){
  if(!dataset)return;
  const {a}=interpolate(time);const pos=new THREE.Vector3(a[1],a[3],-a[2]);
  orbit.target.copy(pos);camera.position.copy(pos).add(new THREE.Vector3(.28,.22,.38));
  previousWorldPos.copy(pos);orbit.update();
}
nativePlay.addEventListener('click',()=>{
  running=!running;nativePlay.textContent=running?'Ⅱ':'▶';
  nativePlay.setAttribute('aria-label',running?'Пауза':'Воспроизвести');
});
nativeSeek.addEventListener('input',()=>{time=Number(nativeSeek.value)||0;draw(time);});
$('reset-camera').addEventListener('click',resetCamera);
async function initialize(){
  try{
    if(!/^wheelbot-v2-brake-(?:short|feedback)-90$/.test(caseId||''))
      throw Error('Неверный идентификатор симуляции');
    state.textContent='Проверяем реальные кадры MuJoCo…';
    dataset=await loadReplay();frames=dataset.frames;
    await makeRobot();
    nativeSeek.max=String(dataset.duration_s);nativeSeek.step='.005';
    time=0;draw(time);resetCamera();
  }catch(e){setError(String(e?.message||e));}
}
function tick(now){
  requestAnimationFrame(tick);
  if(dataset && running && lastAnimation!==null){
    time+=Math.max(0,Math.min(.1,(now-lastAnimation)/1000))*(Number(nativeSpeed.value)||1);
    if(time>dataset.duration_s)time=0;
    draw(time);
  }
  lastAnimation=now;orbit.update();renderer.render(scene,camera);
}
requestAnimationFrame(tick);
initialize();
