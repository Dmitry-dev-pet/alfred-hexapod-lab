import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Kinematic positions from measured Drake plant state + the unmodified Mini Cheetah URDF.
// This viewer NEVER simulates or invents gait; it merely interpolates recorded 50-ms poses.
const q = selector => document.querySelector(selector);
const viewer=q('#viewer'),play=q('#play'),seek=q('#seek'),speed=q('#speed');
const feetLabel=q('#feet'),distLabel=q('#distance'),phaseLabel=q('#phase-label');
const forcesLabel=q('#support-margin'),clock=q('#clock'),error=q('#error');
const follow=q('#follow'),contacts=q('#contacts'),pathToggle=q('#path');
const scene=new THREE.Scene(); scene.background=new THREE.Color('#111c2a');
scene.fog=new THREE.Fog('#111c2a',8,22);
const camera=new THREE.PerspectiveCamera(43,1,.01,40);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.6));
viewer.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.075;orbit.minDistance=.30;orbit.maxDistance=7;orbit.maxPolarAngle=Math.PI*.495;
scene.add(new THREE.HemisphereLight(0xd5eaff,0x3e5268,2.4));
const sun=new THREE.DirectionalLight(0xffffff,2.3);sun.position.set(-1.8,4.5,3.2);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(36,36),new THREE.MeshStandardMaterial({color:0x132231,roughness:.93}));
floor.rotation.x=-Math.PI/2;floor.position.y=-.002;scene.add(floor);
const grid=new THREE.GridHelper(16,160,0x425b6c,0x273b4c);grid.position.y=.002;scene.add(grid);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world); // Drake Z-up -> Three Y-up
const mat=(c)=>new THREE.MeshStandardMaterial({color:c,metalness:.22,roughness:.5});
const shell=mat(0x263a51),detail=mat(0x58a8bd),core=mat(0x93bac6),jointMat=mat(0xd0dfd9);
const loadedMat=mat(0x73e0b8),swingMat=mat(0xf1ae61);
const body=new THREE.Group();world.add(body);
const chassis=new THREE.Mesh(new THREE.BoxGeometry(.36,.14,.075),shell);body.add(chassis);
const inset=new THREE.Mesh(new THREE.BoxGeometry(.24,.11,.013),detail);inset.position.z=.046;body.add(inset);
const cross=new THREE.Mesh(new THREE.BoxGeometry(.41,.022,.025),shell);body.add(cross);
const axis=new THREE.Vector3(0,1,0);
const rodGeom=new THREE.CylinderGeometry(1,1,1,10),jointGeom=new THREE.SphereGeometry(.017,12,8);
function makeRod(radius,material){const o=new THREE.Mesh(rodGeom,material);o.userData.radius=radius;world.add(o);return o;}
function updateRod(o,a,b){const va=new THREE.Vector3(...a),vb=new THREE.Vector3(...b),delta=vb.clone().sub(va),length=Math.max(.0001,delta.length());o.position.copy(va).addScaledVector(delta,.5);o.scale.set(o.userData.radius,length,o.userData.radius);o.quaternion.setFromUnitVectors(axis,delta.normalize());}
const legSegments=Array.from({length:4},()=>[makeRod(.017,core),makeRod(.013,detail),makeRod(.011,core)]);
const spheres=Array.from({length:4},()=>Array.from({length:4},(_,i)=>{const s=new THREE.Mesh(i===3?new THREE.SphereGeometry(.0175,14,9):jointGeom,i===3?loadedMat:jointMat);world.add(s);return s;}));
const supportRings=Array.from({length:4},()=>{const ring=new THREE.Mesh(new THREE.TorusGeometry(.024,.0028,6,20),loadedMat);world.add(ring);return ring;});
let data=null,started=false,running=true,time=0,prev=null,lastCenter=new THREE.Vector3(),pathLine=null;
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
const frameAt=(t)=>Math.min(data.frames.length-1,Math.max(0,t/data.sample_interval_s));
const v=(a,b,i,f)=>a[i]*(1-f)+b[i]*f;
const center=row=>new THREE.Vector3(row[0],row[2],-row[1]);
function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
addEventListener('resize',resize);resize();
function resetCamera(){if(!data)return;const row=data.frames[Math.round(frameAt(time))];const c=center(row);orbit.target.copy(c);const ratio=viewer.clientWidth/Math.max(1,viewer.clientHeight),k=ratio<.9?Math.min(1.75,.9/ratio):1;camera.position.copy(c).add(new THREE.Vector3(.72,.54,.92).multiplyScalar(k));lastCenter.copy(c);orbit.update();}
function applyFrame(t){if(!data)return;const fidx=frameAt(t),ia=Math.floor(fidx),ib=Math.min(ia+1,data.frames.length-1),f=fidx-ia;
 const a=data.frames[ia],b=data.frames[ib];
 const xyz=[v(a,b,0,f),v(a,b,1,f),v(a,b,2,f)],now=new THREE.Vector3(xyz[0],xyz[2],-xyz[1]);
 if(follow.checked){const shift=now.clone().sub(lastCenter);camera.position.add(shift);orbit.target.add(shift);}lastCenter.copy(now);
 body.position.set(...xyz);qa.set(a[4],a[5],a[6],a[3]);qb.set(b[4],b[5],b[6],b[3]);body.quaternion.copy(qa).slerp(qb,f);
 const sampled=data.frames[Math.round(fidx)],fz=sampled.slice(55,59),n=fz.filter(x=>x>data.loaded_threshold_N).length;
 feetLabel.textContent=`${n} / 4`;phaseLabel.textContent=n===2?'Динамическая рысь · 2 опоры':n===3?'Переходная фаза · 3 опоры':n===4?'Контакт всех четырёх ног':'Переход между контактами';
 forcesLabel.textContent=`FL ${fz[0].toFixed(1)} Н · FR ${fz[1].toFixed(1)} Н · HL ${fz[2].toFixed(1)} Н · HR ${fz[3].toFixed(1)} Н`;
 for(let leg=0;leg<4;leg++){const p=[];for(let k=0;k<4;k++){const o=7+leg*12+k*3;p.push([v(a,b,o,f),v(a,b,o+1,f),v(a,b,o+2,f)]);spheres[leg][k].position.set(...p[k]);}
  for(let k=0;k<3;k++)updateRod(legSegments[leg][k],p[k],p[k+1]);
  const touching=fz[leg]>data.loaded_threshold_N;spheres[leg][3].material=touching?loadedMat:swingMat;supportRings[leg].position.set(p[3][0],p[3][1],.005);supportRings[leg].visible=contacts.checked&&touching;}
 distLabel.textContent=(xyz[0]-data.frames[0][0]).toLocaleString('ru-RU',{minimumFractionDigits:2,maximumFractionDigits:2});
 clock.textContent=`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;seek.value=String(t);seek.style.setProperty('--pct',`${100*t/data.duration_s}%`);
}
function toggle(yes){running=yes;play.textContent=running?'Ⅱ':'▶';play.setAttribute('aria-label',running?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>toggle(!running));
seek.addEventListener('input',()=>{time=Number(seek.value);applyFrame(time);});
contacts.addEventListener('change',()=>applyFrame(time));
pathToggle.addEventListener('change',()=>{if(pathLine)pathLine.visible=pathToggle.checked;});
q('#reset-camera').addEventListener('click',resetCamera);
addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT'].includes(document.activeElement?.tagName)){e.preventDefault();toggle(!running);}});
async function load(){try{const res=await fetch('/hexapod/data/cheetah-idto-10s.json',{cache:'no-store'});if(!res.ok)throw Error(`HTTP ${res.status}`);const obj=await res.json();if(obj.format!=='mini-cheetah-idto-physical-replay-v1'||obj.sample_count!==201||obj.frames.length!==201||obj.stride!==60||obj.duration_s!==10||obj.loaded_threshold_N!==.2||obj.frames.some(x=>x.length!==60))throw Error('Некорректный формат записи');data=obj;
 const points=obj.frames.map(x=>new THREE.Vector3(x[0],.009,-x[1]));pathLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0x73dfb5,transparent:true,opacity:.7}));pathLine.visible=pathToggle.checked;scene.add(pathLine);
 seek.max=String(obj.duration_s);resetCamera();applyFrame(0);started=true;
 }catch(e){error.hidden=false;error.textContent='Не удалось загрузить физическую запись: '+e.message;toggle(false);}}
function tick(timestamp){requestAnimationFrame(tick);if(prev!==null&&running&&started){time=(time+(timestamp-prev)/1000*Number(speed.value))%data.duration_s;applyFrame(time);}prev=timestamp;orbit.update();renderer.render(scene,camera);}
requestAnimationFrame(tick);load();
