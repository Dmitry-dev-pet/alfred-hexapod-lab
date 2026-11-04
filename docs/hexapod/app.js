import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const viewer = document.getElementById('viewer');
const seek = document.getElementById('seek');
const playButton = document.getElementById('play');
const speed = document.getElementById('speed');
const followToggle = document.getElementById('follow');
const contactsToggle = document.getElementById('contacts');
const pathToggle = document.getElementById('path');
const clock = document.getElementById('clock');
const feetLabel = document.getElementById('feet');
const distanceLabel = document.getElementById('distance');
const errorLabel = document.getElementById('error');

const scene = new THREE.Scene();
scene.background = new THREE.Color('#111c2a');
scene.fog = new THREE.Fog('#111c2a', 4, 11);
const camera = new THREE.PerspectiveCamera(43, 1, 0.01, 25);
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.6));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.dampingFactor = 0.075;
orbit.minDistance = 0.25;
orbit.maxDistance = 4;
orbit.maxPolarAngle = Math.PI * .495;
orbit.target.set(0, .10, 0);

scene.add(new THREE.HemisphereLight(0xcbe5ff, 0x49505a, 2.6));
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(-1.4, 3.7, 2.5);
scene.add(key);
const fill = new THREE.DirectionalLight(0x78c8e8, 1.0);
fill.position.set(1, 1.1, -2.0);
scene.add(fill);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(25, 25), new THREE.MeshStandardMaterial({ color: '#142231', metalness: .06, roughness: .88 }));
floor.rotation.x = -Math.PI / 2;
floor.position.y = -.005;
scene.add(floor);
const grid = new THREE.GridHelper(8, 80, 0x547086, 0x293d4c);
grid.position.y = .001;
scene.add(grid);

// MuJoCo is Z-up. Only this parent has the coordinate-system transform.
const physicsWorld = new THREE.Group();
physicsWorld.rotation.x = -Math.PI / 2;
scene.add(physicsWorld);
const torso = new THREE.Group();
physicsWorld.add(torso);
const mat = (color, metalness=.25, roughness=.46) => new THREE.MeshStandardMaterial({ color, metalness, roughness });
const carbon = mat(0x243647, .48, .52);
const graphite = mat(0x101e2c, .48, .45);
const mint = mat(0x6ed4b2, .5, .36);
const copper = mat(0xd8a16f, .55, .4);
const light = new THREE.MeshBasicMaterial({color:0xc8ffe5});
function addPart(group, geometry, material, pos, rotation=null) {
  const mesh = new THREE.Mesh(geometry, material);
  if (pos) mesh.position.set(...pos);
  if (rotation) mesh.rotation.set(...rotation);
  group.add(mesh);
  return mesh;
}
// A deliberately original schematic body. This is NOT an exported Freenove STL.
addPart(torso, new THREE.CylinderGeometry(.112,.102,.024,8), carbon, [0,0,.010], [Math.PI/2,0,Math.PI/8]);
addPart(torso, new THREE.BoxGeometry(.135,.142,.026), graphite, [0,0,.032]);
addPart(torso, new THREE.BoxGeometry(.100,.087,.010), mint, [0,.005,.049]);
for (const x of [-.044,.044]) {
  addPart(torso, new THREE.BoxGeometry(.029,.051,.018), copper, [x,-.036,.061]);
  addPart(torso, new THREE.SphereGeometry(.008,10,7), light, [x,.106,.030]);
}
addPart(torso, new THREE.BoxGeometry(.064,.026,.030), carbon, [0,.114,.027]);

const rodMaterials = [mat(0x91bbcd), mat(0x79c6c3), mat(0x6ca49f)];
const jointMaterial = mat(0xb6d0da,.5,.35);
const downMaterial = new THREE.MeshStandardMaterial({color:0x7befb1,emissive:0x125e43,emissiveIntensity:.6});
const upMaterial = new THREE.MeshStandardMaterial({color:0xe9a35c,emissive:0x5e3613,emissiveIntensity:.4});
const legGeometry = [new THREE.CylinderGeometry(.011,.011,1,9), new THREE.CylinderGeometry(.009,.008,1,9), new THREE.CylinderGeometry(.007,.005,1,9)];
const jointGeo = new THREE.SphereGeometry(.014,12,9);
const toeGeo = new THREE.SphereGeometry(.012,12,9);
const legs = [];
for (let i=0;i<6;i++) {
  const rods = legGeometry.map((geom,j) => addPart(physicsWorld,geom,rodMaterials[j],null));
  const joints = [0,1,2].map(() => addPart(physicsWorld,jointGeo,jointMaterial,null));
  const toe = addPart(physicsWorld,toeGeo,downMaterial,null);
  const ring = addPart(physicsWorld,new THREE.TorusGeometry(.021,.0025,6,20),downMaterial,null);
  legs.push({rods,joints,toe,ring});
}
const axisY = new THREE.Vector3(0,1,0);
const pointA = new THREE.Vector3(), pointB = new THREE.Vector3(), delta = new THREE.Vector3();
function updateRod(rod,a,b) {
  pointA.set(...a); pointB.set(...b);
  delta.subVectors(pointB,pointA);
  const length = Math.max(0.0001,delta.length());
  rod.position.copy(pointA).addScaledVector(delta,.5);
  rod.scale.set(1,length,1);
  rod.quaternion.setFromUnitVectors(axisY,delta.normalize());
}
const tmpRoot = new THREE.Vector3();
const tmpQuatA = new THREE.Quaternion(), tmpQuatB = new THREE.Quaternion();
const tmpQuat = new THREE.Quaternion();
let lastBodyWorld = new THREE.Vector3();
let trajectory = null;
let t = 0;
let running = true;
let prevTime = null;
let pathLine;

function readPoint(frame,offset) { return [frame[offset],frame[offset+1],frame[offset+2]]; }
function lerpPoint(a,b,offset,alpha) { return [0,1,2].map(k=>a[offset+k]*(1-alpha)+b[offset+k]*alpha); }
function rootCenter(frame) { return new THREE.Vector3(frame[0],frame[2],-frame[1]); }
function resetCamera() {
  if (!trajectory) return;
  const s = trajectory.frames[Math.max(0,Math.min(trajectory.frames.length-1,Math.round(t/trajectory.sample_interval_s)))];
  const c = rootCenter(s);
  orbit.target.copy(c);
  camera.position.copy(c).add(new THREE.Vector3(.49,.34,.63));
  lastBodyWorld.copy(c);
  orbit.update();
}
function applyFrame(time) {
  const f = trajectory.frames;
  const at = Math.max(0,Math.min(f.length-1,time/trajectory.sample_interval_s));
  const ia = Math.floor(at), ib = Math.min(ia+1,f.length-1), alpha = at-ia;
  const a=f[ia],b=f[ib];
  const root = lerpPoint(a,b,0,alpha);
  torso.position.set(...root);
  tmpQuatA.set(a[4],a[5],a[6],a[3]);
  tmpQuatB.set(b[4],b[5],b[6],b[3]);
  tmpQuat.copy(tmpQuatA).slerp(tmpQuatB,alpha);
  torso.quaternion.copy(tmpQuat);
  const mask = alpha < .5 ? a[79] : b[79];
  let grounded = 0;
  for (let i=0;i<6;i++) {
    const points = [];
    for (let j=0;j<4;j++) points.push(lerpPoint(a,b,7+i*12+j*3,alpha));
    const l = legs[i];
    for (let j=0;j<3;j++) updateRod(l.rods[j],points[j],points[j+1]);
    for (let j=0;j<3;j++) l.joints[j].position.set(...points[j]);
    l.toe.position.set(...points[3]);
    l.ring.position.set(points[3][0],points[3][1],Math.max(.004,points[3][2]));
    const contact = !!(mask & (1<<i));
    grounded += Number(contact);
    l.toe.material = contact?downMaterial:upMaterial;
    l.ring.material = contact?downMaterial:upMaterial;
    l.ring.visible = contactsToggle.checked && contact;
  }
  feetLabel.textContent = `${grounded} / 6`;
  const fwd = trajectory.forward_xy;
  const start = trajectory.frames[0];
  const d = (root[0]-start[0])*fwd[0] + (root[1]-start[1])*fwd[1];
  distanceLabel.textContent = d.toLocaleString('ru-RU',{minimumFractionDigits:2,maximumFractionDigits:2});
  // Follow the physical body translation without overriding manual orbit angle.
  const nowWorld = new THREE.Vector3(root[0],root[2],-root[1]);
  if (followToggle.checked) {
    const shift = nowWorld.clone().sub(lastBodyWorld);
    camera.position.add(shift);
    orbit.target.add(shift);
  }
  lastBodyWorld.copy(nowWorld);
  clock.textContent = `00:${String(Math.floor(time)).padStart(2,'0')}`;
  seek.value = String(time);
  seek.style.setProperty('--pct',`${time/trajectory.duration_s*100}%`);
}
function togglePlay(yes) {
  running=yes;
  playButton.textContent=running?'Ⅱ':'▶';
  playButton.setAttribute('aria-label',running?'Пауза':'Воспроизвести');
}
playButton.addEventListener('click',()=>togglePlay(!running));
seek.addEventListener('input',()=>{t=Number(seek.value);if(trajectory)applyFrame(t);});
document.getElementById('reset-camera').addEventListener('click',resetCamera);
pathToggle.addEventListener('change',()=>{if(pathLine)pathLine.visible=pathToggle.checked;});
contactsToggle.addEventListener('change',()=>{if(trajectory)applyFrame(t);});
window.addEventListener('keydown',e=>{if((e.code==='Space'||e.key===' ')&& !['INPUT','SELECT'].includes(document.activeElement?.tagName)){e.preventDefault();togglePlay(!running);}});
function resize() {
  const w=Math.max(1,viewer.clientWidth), h=Math.max(1,viewer.clientHeight);
  camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);
}
window.addEventListener('resize',resize);
resize();

async function load() {
  try {
    const res=await fetch('./data/tripod-60s.json');
    if(!res.ok) throw new Error(`HTTP ${res.status}`);
    const obj=await res.json();
    if(obj.format!=='fnk0052-physical-points-v1'||obj.stride!==80||obj.frames.length!==obj.sample_count||obj.duration_s<59.9)
      throw new Error('Неподдерживаемый формат траектории');
    trajectory=obj;
    seek.max=String(obj.duration_s);
    const path=obj.frames.map(frame=>new THREE.Vector3(frame[0],.008,-frame[1]));
    const geo=new THREE.BufferGeometry().setFromPoints(path);
    pathLine=new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x63dcae,transparent:true,opacity:.67}));
    pathLine.visible=pathToggle.checked;
    scene.add(pathLine);
    resetCamera();
    applyFrame(0);
  } catch (err) {
    errorLabel.hidden=false;
    errorLabel.textContent=`Не удалось загрузить траекторию: ${err.message}. Откройте страницу по HTTPS на GitHub Pages или через локальный HTTP-сервер.`;
    togglePlay(false);
  }
}
function animate(timestamp) {
  requestAnimationFrame(animate);
  if(prevTime!==null && running && trajectory) {
    t=Math.min(trajectory.duration_s,t+(timestamp-prevTime)/1000*Number(speed.value));
    if(t===trajectory.duration_s) togglePlay(false);
    applyFrame(t);
  }
  prevTime=timestamp;
  orbit.update();
  renderer.render(scene,camera);
}
requestAnimationFrame(animate);
load();
