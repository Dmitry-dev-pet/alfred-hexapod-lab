import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

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
const cadStatus = document.getElementById('cad-status');
const cadToggle = document.getElementById('show-cad');
const phaseLabel = document.getElementById('phase-label');
const supportMarginLabel = document.getElementById('support-margin');

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
const schematicGroup = new THREE.Group();
physicsWorld.add(schematicGroup);
const cadRoot = new THREE.Group();
physicsWorld.add(cadRoot);
cadRoot.visible = false;
const torso = new THREE.Group();
schematicGroup.add(torso);
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
  const rods = legGeometry.map((geom,j) => addPart(schematicGroup,geom,rodMaterials[j],null));
  const joints = [0,1,2].map(() => addPart(schematicGroup,jointGeo,jointMaterial,null));
  const toe = addPart(schematicGroup,toeGeo,downMaterial,null);
  const ring = addPart(schematicGroup,new THREE.TorusGeometry(.021,.0025,6,20),downMaterial,null);
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
let cadRig=null;
let cadGroups=[];
let cadLoaded=0;
let cadRigReady=false;

function readPoint(frame,offset) { return [frame[offset],frame[offset+1],frame[offset+2]]; }
function lerpPoint(a,b,offset,alpha) { return [0,1,2].map(k=>a[offset+k]*(1-alpha)+b[offset+k]*alpha); }
function rootCenter(frame) { return new THREE.Vector3(frame[0],frame[2],-frame[1]); }
function resetCamera() {
  if (!trajectory) return;
  const s = trajectory.frames[Math.max(0,Math.min(trajectory.frames.length-1,Math.round(t/trajectory.sample_interval_s)))];
  const c = rootCenter(s);
  orbit.target.copy(c);
  // Portrait tablets have a narrower horizontal field of view.
  const ratio=viewer.clientWidth/Math.max(1,viewer.clientHeight);
  const portraitScale=ratio<.9?Math.min(1.85,.9/ratio):1;
  camera.position.copy(c).add(new THREE.Vector3(.49,.34,.63).multiplyScalar(portraitScale));
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
  if (trajectory.phase_samples) {
    const ix=Math.min(trajectory.phase_samples.length-1,Math.round(time/trajectory.sample_interval_s));
    const phases={initial_prep:'Подготовка опор',lift:'Подъём двух клешней',choose:'Выбор опорной ноги',step:'Физический перенос ноги',touchdown:'Приземление задней ноги',recover:'Возврат на шесть опор'};
    let label=phases[trajectory.phase_samples[ix]]||trajectory.phase_samples[ix];
    const ev=trajectory.events||[];
    const open=ev.filter(v=>v.event==='swing_approved' && v.t<=time+0.002).at(-1);
    if (open && !ev.some(v=>v.event==='touchdown_checked' && v.leg===open.leg && v.t<=time+0.002)) label+=` · ${open.leg==='bl'?'левая задняя':'правая задняя'}`;
    phaseLabel.textContent=label;
    const sample=trajectory.support_samples?.[ix];
    if (sample) supportMarginLabel.textContent=`Опорный запас: ${sample.margin_mm.toFixed(1)} мм · момент: ${sample.torque_nm.toFixed(2)} Н·м`;
  }
  clock.textContent = `${String(Math.floor(time/60)).padStart(2,'0')}:${String(Math.floor(time%60)).padStart(2,'0')}`;
  seek.value = String(time);
  seek.style.setProperty('--pct',`${time/trajectory.duration_s*100}%`);
  updateCAD(time);
}
// Browser uses the *original* 30 Freenove STL shapes directly from the
// pinned public GitHub upstream, but this site never redistributes CAD binaries.
// World body poses come from mj_forward on the original 60 s qpos recording.
function wxyz(q) { return new THREE.Quaternion(q[1],q[2],q[3],q[0]); }
function refreshCADVisibility() {
  const active = cadRigReady && cadToggle.checked && cadLoaded >= 18;
  cadRoot.visible = active;
  schematicGroup.visible = !active;
}
function updateCAD(time) {
  if (!cadRigReady || !cadRig) return;
  const f = cadRig.frames;
  const at = Math.max(0,Math.min(f.length-1,time/cadRig.sample_interval_s));
  const ia = Math.floor(at), ib = Math.min(ia+1,f.length-1), alpha = at-ia;
  const a = f[ia], b = f[ib];
  for (let i=0; i<cadGroups.length; i++) {
    const o=i*7, mesh=cadGroups[i];
    mesh.position.set(
      a[o]*(1-alpha)+b[o]*alpha,
      a[o+1]*(1-alpha)+b[o+1]*alpha,
      a[o+2]*(1-alpha)+b[o+2]*alpha
    );
    tmpQuatA.set(a[o+4],a[o+5],a[o+6],a[o+3]);
    tmpQuatB.set(b[o+4],b[o+5],b[o+6],b[o+3]);
    mesh.quaternion.copy(tmpQuatA).slerp(tmpQuatB,alpha);
  }
}
function cadMaterial(rgba) {
  const a=rgba[3];
  return new THREE.MeshStandardMaterial({
    color:new THREE.Color(rgba[0],rgba[1],rgba[2]),
    metalness:.22, roughness:.58,
    side:THREE.DoubleSide,
    transparent:a < .995, opacity:a, depthWrite:a >= .995
  });
}
function cadFallbackGeometry(g) {
  if(g.kind==='cylinder') {
    const geometry=new THREE.CylinderGeometry(g.size[0],g.size[0],g.size[1]*2,16);
    geometry.rotateX(Math.PI/2); // Three.js cylinder is Y-up, MuJoCo is Z-up.
    return geometry;
  }
  return new THREE.BoxGeometry(g.size[0]*2,g.size[1]*2,g.size[2]*2);
}
async function loadCAD() {
  try {
    cadStatus.textContent='Загрузка оригинального CAD…';
    const r=await fetch('../data/cad-rig-claws-10s.json');
    if(!r.ok) throw new Error(`CAD manifest HTTP ${r.status}`);
    const rig=await r.json();
    if(rig.format!=='fnk0052-mujoco-cad-visual-replay-v1'||rig.meshes.length!==30||rig.bodies.length!==27||rig.frames.length!==100||rig.frame_stride!==189)
      throw new Error('Некорректный формат модели CAD30');
    cadRig=rig;
    cadGroups=rig.bodies.map(b=>{
      const node=new THREE.Group(); node.name=b.name;cadRoot.add(node);return node;
    });
    const byBody=new Map(rig.bodies.map((b,i)=>[b.id,cadGroups[i]]));
    const loader=new STLLoader();
    const geometries=new Map();
    const errors=[];
    // A modest number of simultaneous requests avoids saturating tablet RAM.
    let next=0;
    async function worker() {
      while(next<rig.meshes.length) {
        const mesh=rig.meshes[next++];
        try {
          const geometry=await loader.loadAsync(mesh.url);
          // MuJoCo uses centered/oriented mesh vertices. Undo its mesh frame
          // from the upstream STL, preserving the true original shape.
          geometry.translate(-mesh.mesh_pos[0],-mesh.mesh_pos[1],-mesh.mesh_pos[2]);
          geometry.applyQuaternion(wxyz(mesh.mesh_quat_wxyz).invert());
          geometry.computeBoundingSphere();
          geometries.set(mesh.id,geometry);
          cadLoaded++;
          cadStatus.textContent=`CAD: ${cadLoaded} / ${rig.mesh_count} STL`;
        } catch(e) {
          errors.push(`${mesh.filename}: ${e.message}`);
        }
      }
    }
    await Promise.all(Array.from({length:5},()=>worker()));
    const materialCache=new Map();
    for (const g of rig.geoms) {
      const parent=byBody.get(g.body);
      if(!parent)throw new Error(`CAD body ${g.body} missing`);
      const local=new THREE.Group();
      local.position.set(...g.pos);
      local.quaternion.copy(wxyz(g.quat_wxyz));
      const mk=g.rgba.join(',');
      if(!materialCache.has(mk)) materialCache.set(mk,cadMaterial(g.rgba));
      const material=materialCache.get(mk);
      const geom=g.kind==='mesh' ? geometries.get(g.mesh) : cadFallbackGeometry(g);
      if(!geom)continue;
      const part=new THREE.Mesh(geom,material);
      part.name=g.name;
      local.add(part);parent.add(local);
    }
    cadRigReady=true;
    refreshCADVisibility();
    if(trajectory)updateCAD(t);
    cadStatus.textContent=cadLoaded===rig.mesh_count
      ? `Оригинальный CAD: ${cadLoaded}/30 STL · 2 заменены`
      : `CAD: ${cadLoaded}/30 STL · ${errors.length} недоступно`;
    if(errors.length) console.warn('CAD STL failed to load from original upstream',errors);
  } catch(e) {
    cadStatus.textContent=`CAD недоступен: ${e.message}`;
    cadRigReady=false;
    refreshCADVisibility();
    console.warn('CAD rig load failed',e);
  }
}
cadToggle.addEventListener('change',refreshCADVisibility);

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
    const res=await fetch('../data/claws-10s.json');
    if(!res.ok) throw new Error(`HTTP ${res.status}`);
    const obj=await res.json();
    if(obj.format!=='fnk0052-physical-points-v1'||obj.stride!==80||obj.frames.length!==obj.sample_count||obj.duration_s<9.8)
      throw new Error('Неподдерживаемый формат траектории');
    if (obj.summary?.full_four_leg_gait!==false || obj.summary?.new_leg_cycle_coverage!==2) throw new Error('Неверная классификация исследования');
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
    t=(t+(timestamp-prevTime)/1000*Number(speed.value))%trajectory.duration_s;
    // Keep the recorded physical motion looping, as in a video player.
    applyFrame(t);
  }
  prevTime=timestamp;
  orbit.update();
  renderer.render(scene,camera);
}
requestAnimationFrame(animate);
load();
loadCAD();
