import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

// Browser playback ONLY: each displayed pose comes from a saved MuJoCo qpos.
// The schematic solids are drawn from the upstream model dimensions; upstream
// third-party code, XML and assets are not copied into this player.
const $=id=>document.getElementById(id);
const ID=document.querySelector('.layout')?.dataset?.caseId||'';
const SOURCE='7bbc38f4b26b8ecce7ac0ee37e06164e851dc600';
const CASES=new Set(['sebaju-jump-v1','sebaju-tip12-v1','sebaju-tip20-long-v1']);
const stage=$('viewer'),play=$('play'),seek=$('seek'),speed=$('speed');
const status=$('state'),pose=$('pose'),contacts=$('contacts'),error=$('error');
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5));
renderer.outputColorSpace=THREE.SRGBColorSpace;
stage.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x10202c);
const camera=new THREE.PerspectiveCamera(44,1,.005,35);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;orbit.dampingFactor=.09;orbit.minDistance=.23;orbit.maxDistance=6;
scene.add(new THREE.HemisphereLight(0xdbe9fa,0x344f66,2.6));
const key=new THREE.DirectionalLight(0xffffff,2.8);key.position.set(1,2,2.5);scene.add(key);
const fill=new THREE.DirectionalLight(0x8ec7e8,1.1);fill.position.set(-2,1,-2);scene.add(fill);
// All model coordinates are MuJoCo Z-up, including the ground plane.
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(16,16),new THREE.MeshStandardMaterial({color:0x203541,roughness:.93,side:THREE.DoubleSide}));
floor.position.z=-.002;world.add(floor);
const grid=new THREE.GridHelper(8,80,0x53768a,0x314b5b);grid.rotation.x=Math.PI/2;world.add(grid);
const robot=new THREE.Group();world.add(robot);
const matBody=new THREE.MeshStandardMaterial({color:0x9babb9,metalness:.2,roughness:.55});
const matBallast=new THREE.MeshStandardMaterial({color:0x444d55,metalness:.15,roughness:.7});
const matLeg=new THREE.MeshStandardMaterial({color:0xe35c45,metalness:.15,roughness:.5});
const matWheel=new THREE.MeshStandardMaterial({color:0x17202a,metalness:.15,roughness:.9});
const matRim=new THREE.MeshStandardMaterial({color:0x70c4a5,metalness:.5,roughness:.38});
const matStripe=new THREE.MeshStandardMaterial({color:0xffce81,metalness:.1,roughness:.45});
function box(x,y,z,w,h,d,mat,parent){
 const mesh=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),mat);
 mesh.position.set(x,y,z);parent.add(mesh);return mesh;
}
function segment(a,b,r,mat,parent){
 const A=new THREE.Vector3(...a),B=new THREE.Vector3(...b),v=B.clone().sub(A);
 const mesh=new THREE.Mesh(new THREE.CylinderGeometry(r,r,v.length(),14),mat);
 mesh.position.copy(A).add(B).multiplyScalar(.5);
 mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),v.normalize());parent.add(mesh);
 for(const p of [A,B]){const cap=new THREE.Mesh(new THREE.SphereGeometry(r,12,8),mat);cap.position.copy(p);parent.add(cap);}
}
// Original model torso: 150 x 190 x 110 mm. Left/right limb lengths 130 mm.
box(.025,0,0,.15,.19,.11,matBody,robot);
box(.045,0,-.045,.07,.14,.036,matBallast,robot);
const wheelJoints=[];
const mechanisms=[];
for(const side of [1,-1]){
 const hip=new THREE.Group();hip.position.set(0,side*.095,0);robot.add(hip);
 segment([0,0,0],[-.0919,0,-.0919],.014,matLeg,hip);
 const knee=new THREE.Group();knee.position.set(-.0919,0,-.0919);hip.add(knee);
 segment([0,0,0],[.0919,0,-.0919],.012,matLeg,knee);
 const wheel=new THREE.Group();wheel.position.set(.0919,side*.052,-.0919);knee.add(wheel);
 const tyre=new THREE.Mesh(new THREE.CylinderGeometry(.06,.06,.056,36),matWheel);wheel.add(tyre);
 const rim=new THREE.Mesh(new THREE.CylinderGeometry(.026,.026,.059,28),matRim);wheel.add(rim);
 for(let theta=0;theta<Math.PI*2;theta+=Math.PI/3){
  const marker=new THREE.Mesh(new THREE.SphereGeometry(.006,10,7),matStripe);
  marker.position.set(.043*Math.cos(theta),side*.031,.043*Math.sin(theta));wheel.add(marker);
 }
 mechanisms.push({hip,knee,wheel,side});wheelJoints.push(wheel);
}
// mech[0] = left, mech[1] = right; model joint order is left hip/knee/wheel, right hip/knee/wheel.
const q=new THREE.Quaternion();
let current=null,rows=[],time=0,lastNow=null,playing=true,following=null,lastDrawn=-1;
function resize(){const w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function fail(msg){error.hidden=false;error.textContent=msg;status.textContent='Не удалось открыть запись';playing=false;play.textContent='▶';play.setAttribute('aria-label','Воспроизвести');}
async function sha256(buf){
 const arr=new Uint8Array(await crypto.subtle.digest('SHA-256',buf));
 return [...arr].map(x=>x.toString(16).padStart(2,'0')).join('');
}
async function load(){
 if(!CASES.has(ID))throw Error('Неизвестная запись SeBaJu');
 if(!globalThis.crypto?.subtle||!('DecompressionStream' in window))throw Error('Нужен современный Chrome с SHA-256 и gzip');
 const response=await fetch('/hexapod/data/'+ID+'.json',{cache:'no-store'});
 if(!response.ok)throw Error('Нет данных SeBaJu: HTTP '+response.status);
 const pack=await response.json();
 if(pack.format!=='sebaju-mujoco-gzip-b64-v1'||pack.case_id!==ID||pack.compression!=='gzip'||
    pack.source_commit!==SOURCE||pack.engine!=='MuJoCo 3.14.0'||!Number.isInteger(pack.frame_count)||
    typeof pack.base64!=='string'||!Number.isFinite(pack.duration_s)||pack.duration_s<=0)
    throw Error('Неверный формат или происхождение записи');
 const gz=Uint8Array.from(atob(pack.base64),c=>c.charCodeAt(0));
 const raw=await new Response(new Blob([gz]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
 if(raw.byteLength!==pack.raw_bytes || await sha256(raw)!==pack.raw_sha256)
    throw Error('SHA-256 физических кадров не совпадает');
 const data=JSON.parse(new TextDecoder().decode(raw));
 if(data.format!=='sebaju-mujoco-measured-v1'||data.case_id!==ID||data.source_commit!==SOURCE||
    data.engine!=='MuJoCo 3.14.0'||data.source_trace_sha256!==pack.source_trace_sha256||
    data.source_receipt_sha256!==pack.source_receipt_sha256||
    data.frame_count!==pack.frame_count||data.duration_s!==pack.duration_s||
    !Array.isArray(data.rows)||data.rows.length!==pack.frame_count)
    throw Error('Данные MuJoCo не прошли проверку источника');
 if((ID==='sebaju-jump-v1' && (!data.summary.jump_detected||!data.summary.landing_recovery_observed)) ||
    (ID!=='sebaju-jump-v1'&&(!data.summary.physical_tip_observed||data.summary.self_righted!==false)))
    throw Error('Результат не совпадает с протоколом эксперимента');
 let prev=-Infinity;
 for(const row of data.rows){
   if(!Array.isArray(row)||row.length!==21||row.some(x=>typeof x!=='number'||!Number.isFinite(x))||row[0]<=prev)
      throw Error('Кадры повреждены или идут не по времени');
   const quatSq=row[4]**2+row[5]**2+row[6]**2+row[7]**2;
   if(Math.abs(quatSq-1)>.02)throw Error('Неверный кватернион кадра');
   prev=row[0];
 }
 if(Math.abs(prev-data.duration_s)>.03)throw Error('Запись обрывается до конца');
 return data;
}
function rowAt(t){
 if(t<=rows[0][0])return 0;
 if(t>=rows.at(-1)[0])return rows.length-1;
 let lo=0,hi=rows.length-1;
 while(hi-lo>1){const mid=(lo+hi)>>1;if(rows[mid][0]<=t)lo=mid;else hi=mid;}
 return lo; // measured sample; no interpolated or invented physics frames.
}
function mappedRoot(row){return new THREE.Vector3(row[1],row[3],-row[2]);}
function show(t){
 if(!current)return;
 const idx=rowAt(t);if(idx===lastDrawn)return;lastDrawn=idx;
 const row=rows[idx];
 robot.position.set(row[1],row[2],row[3]);
 q.set(row[5],row[6],row[7],row[4]).normalize();robot.quaternion.copy(q);
 mechanisms[0].hip.rotation.y=row[8];mechanisms[0].knee.rotation.y=row[9];mechanisms[0].wheel.rotation.y=row[10]%(2*Math.PI);
 mechanisms[1].hip.rotation.y=row[11];mechanisms[1].knee.rotation.y=row[12];mechanisms[1].wheel.rotation.y=row[13]%(2*Math.PI);
 const nowPos=mappedRoot(row);
 if(!following)following=nowPos.clone();
 else {const delta=nowPos.clone().sub(following);camera.position.add(delta);orbit.target.add(delta);following.copy(nowPos);}
 const tilt=row[14],tSample=row[0],contactMask=row[19];
 const jumpStart=current.summary.takeoff_s||5.8,touch=current.summary.touchdown_s||6.0;
 const tipTime=current.summary.tip_detected_s;
 const sceneName=ID==='sebaju-jump-v1'?
  (tSample<5?'Балансирование':tSample<jumpStart?'Подготовка к прыжку':tSample<touch?'ПОЛЁТ':tSample<8.2?'Приземление':'Баланс восстановлен'):
  (tSample<12?'Прыжок и баланс':tSample<(tipTime??13)?'Боковой импульс':tSample<15?'Падение':'Не поднялся');
 status.textContent=sceneName;
 pose.textContent=tSample.toFixed(2)+' / '+current.duration_s.toFixed(0)+' с · наклон '+tilt.toFixed(1)+'°';
 const wheelContact=row[15]+row[16],geomCount=contactMask.toString(2).replace(/0/g,'').length;
 contacts.textContent='Контакт колёс '+wheelContact+'/2 · контактных частей '+geomCount+
   ' · Fz '+(row[17]+row[18]).toFixed(1)+' Н';
 seek.value=String(tSample);
}
function resetCamera(){
 if(!current)return;
 const p=mappedRoot(rows[rowAt(time)]);
 following=p.clone();orbit.target.copy(p).add(new THREE.Vector3(0,.075,0));
 camera.position.copy(p).add(new THREE.Vector3(.53,.37,.63));orbit.update();
}
play.addEventListener('click',()=>{playing=!playing;play.textContent=playing?'Ⅱ':'▶';play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');});
seek.addEventListener('input',()=>{time=Number(seek.value)||0;lastDrawn=-1;show(time);});
$('reset-camera').addEventListener('click',resetCamera);
async function start(){
 try{
   status.textContent='Проверка SHA-256 и физических кадров…';
   current=await load();rows=current.rows;seek.max=String(current.duration_s);seek.step='.005';time=0;show(time);resetCamera();
 }catch(e){fail(String(e?.message||e));console.error('SeBaJu player:',e);}
}
function animate(now){
 requestAnimationFrame(animate);
 if(current && playing && lastNow!==null){
  time+=Math.min(Math.max(0,(now-lastNow)/1000),.08)*(Number(speed.value)||1);
  if(time>current.duration_s){time=0;lastDrawn=-1;}
  show(time);
 }
 lastNow=now;orbit.update();renderer.render(scene,camera);
}
requestAnimationFrame(animate);start();
