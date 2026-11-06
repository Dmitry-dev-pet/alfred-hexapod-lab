import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
// DISPLAY ONLY. Both trajectories are recorded MuJoCo qpos -> pinned body FK.
// Never alter physical root coordinates, interpolate new commands, or simulate in JS.
const $=s=>document.querySelector(s);
const COMMIT='447945fd40fb5585271dce4ea8cc82567916032a';
const ROOT='https://raw.githubusercontent.com/LuwuDynamics/luwu_mjlab/'+COMMIT+'/src/assets/robots/lite3/xmls/assets/';
const MANIFEST='/hexapod/lite3-original-v1/asset-manifest.json';
const CACHE='hexapod-lite3-original-'+COMMIT;
const CACHE_KEY='/hexapod/.private-lite3-mesh-cache/';
const files={
 original_baseline:'lite3-s2-original_baseline-seed20261106-v1.json',
 friction_035:'lite3-s2-friction_035-seed20261106-v1.json',
 friction_020:'lite3-s2-friction_020-seed20261106-v1.json',
 uniform_mass_0610:'lite3-s2-uniform_mass_0610-seed20261106-v1.json',
 actuator_cap_02256:'lite3-s2-actuator_cap_02256-seed20261106-v1.json',
 combined_stress:'lite3-s2-combined_stress-seed20261106-v1.json'
};
// Independently recorded 10-seed aggregate metrics, S2_60RUN_AGGREGATE.json.
const stats={original_baseline:{slip:12.9,sat:27.6,forward:439},
 friction_035:{slip:18.9,sat:27.2,forward:441},friction_020:{slip:23.3,sat:26.9,forward:438},
 uniform_mass_0610:{slip:15.1,sat:30.1,forward:379},
 actuator_cap_02256:{slip:13.5,sat:41.7,forward:393},
 combined_stress:{slip:22.1,sat:43.7,forward:321}};
const BASE_COLOR=0x56dcf0,VAR_COLOR=0xffbd76;
const stage=$('#scene');const notice=$('#notice'),error=$('#err');
let baseline=null,experiment=null,spec=null,geometries=[],playing=false,t=0,lastFrame=null;
let mode='overlay',plane='isometric',loadSerial=0,prevPair=null,baseMeshes=[],otherMeshes=[],baseMarkers=[],otherMarkers=[];
let baseTrail=null,otherTrail=null,referenceLine=null,ready=false,chartTransform=null;
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.setClearColor(0x101e2b,1);stage.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color('#101e2b');
const camera=new THREE.PerspectiveCamera(47,1,.01,15);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.07;
orbit.minDistance=.25;orbit.maxDistance=4;orbit.maxPolarAngle=Math.PI*.51;
scene.add(new THREE.HemisphereLight(0xe9f8ff,0x4b6686,2.6));
const keyLight=new THREE.DirectionalLight(0xffffff,2.3);keyLight.position.set(-1,3,4);scene.add(keyLight);
const fillLight=new THREE.DirectionalLight(0x8dd9fa,1.1);fillLight.position.set(3,1,-2);scene.add(fillLight);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(15,15),new THREE.MeshStandardMaterial({color:0x1b2d3b,roughness:.9}));floor.rotation.x=-Math.PI/2;floor.position.y=-.002;scene.add(floor);
const grid=new THREE.GridHelper(8,80,0x406879,0x233c4d);grid.position.y=.001;scene.add(grid);
// MuJoCo world is Z-up. All transforms below stay in MJ XYZ frame.
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const baseGroup=new THREE.Group(),otherGroup=new THREE.Group();world.add(baseGroup,otherGroup);
const loader=new STLLoader();
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
function resize(){let w=Math.max(1,stage.clientWidth),h=Math.max(1,stage.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function cameraReset(){const center=new THREE.Vector3(.17,.065,-.19);orbit.target.copy(center);
  const portrait=camera.aspect<1?Math.min(1.6,1/camera.aspect):1;
  camera.position.copy(center).add(new THREE.Vector3(.43,.52,.66).multiplyScalar(portrait));orbit.update();prevPair=null;
}
function cameraTop(){const center=new THREE.Vector3(.17,0,-.19);orbit.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(.01,1.12,.01));orbit.update();plane='top';prevPair=null;}
function notify(message,err=false){notice.hidden=err;error.hidden=!err;if(err)error.textContent=message;else notice.textContent=message;}
function bufferDigest(data){return crypto.subtle.digest('SHA-256',data).then(d=>[...new Uint8Array(d)].map(b=>b.toString(16).padStart(2,'0')).join(''));}
async function verify(buf,mesh){if(buf.byteLength!==mesh.size||await bufferDigest(buf)!==mesh.sha256)throw Error('Несовпадение исходной STL: '+mesh.mesh);return buf;}
function validateManifest(m){if(m.schema!=='xgo-lite3-original-local-stl-assets-v1'||m.commit!==COMMIT||!Array.isArray(m.meshes)||m.meshes.length!==17)throw Error('Некорректный манифест STL');
  for(const s of m.meshes){if(!/^[A-Za-z][\w]*\.STL$/.test(s.mesh)||!/^[0-9a-f]{64}$/.test(s.sha256)||!Number.isInteger(s.size)||s.size<1000||s.size>5000000)throw Error('Недопустимая запись STL');}return m;
}
async function getMesh(mesh,cache){const local='/assets/lite3-original/'+mesh.mesh;
  try{const r=await fetch(local,{cache:'no-store'});if(r.ok)return await verify(await r.arrayBuffer(),mesh);}catch(e){if(e.message.startsWith('Несовпадение'))throw e;}
  const key=CACHE_KEY+mesh.mesh;
  if(cache){const cached=await cache.match(key);if(cached){try{return await verify(await cached.arrayBuffer(),mesh);}catch{await cache.delete(key);}}}
  const r=await fetch(ROOT+mesh.mesh,{mode:'cors',cache:'no-store'});if(!r.ok)throw Error('STL недоступна: '+mesh.mesh+' HTTP '+r.status);
  const data=await verify(await r.arrayBuffer(),mesh);
  if(cache)try{await cache.put(key,new Response(data.slice(0),{headers:{'Content-Type':'application/octet-stream'}}));}catch{};
  return data;
}
function colorFor(name){if(name==='body')return 0x2f3943;if(name==='mainarm'||name==='forearm')return 0x788894;
 if(name.toLowerCase().includes('claw')||name.toLowerCase().includes('calw'))return 0x929ba6;
 if(name.endsWith('_hip')||name.endsWith('_shank'))return 0x93a3ae;
 return 0x40505d;
}
function footMarker(group,color){const g=new THREE.Mesh(new THREE.TorusGeometry(.012,.0023,5,20),new THREE.MeshBasicMaterial({color,transparent:true,opacity:.9,depthTest:false}));
 g.visible=false;g.renderOrder=9;group.add(g);return g;}
async function loadSTLs(){const r=await fetch(MANIFEST,{cache:'no-store'});if(!r.ok)throw Error('Нет манифеста STL');spec=validateManifest(await r.json());
 let cache=null;if('caches' in window)try{cache=await caches.open(CACHE);}catch{};
 for(let i=0;i<spec.meshes.length;i++){const item=spec.meshes[i];notify(`Загружаем оригинальные STL ${i+1}/17…`);
  const bytes=await getMesh(item,cache);const geom=loader.parse(bytes);geom.computeVertexNormals();geometries.push(geom);
  // The translucent teal wireframe intentionally marks the baseline, not physics.
  const ghost=new THREE.Mesh(geom,new THREE.MeshBasicMaterial({color:BASE_COLOR,wireframe:true,transparent:true,opacity:.5,depthTest:false,depthWrite:false}));
  ghost.renderOrder=6;ghost.frustumCulled=false;baseGroup.add(ghost);baseMeshes.push(ghost);
  const subject=new THREE.Mesh(geom,new THREE.MeshStandardMaterial({color:colorFor(item.body),metalness:.23,roughness:.47,side:THREE.DoubleSide}));
  subject.frustumCulled=false;otherGroup.add(subject);otherMeshes.push(subject);
 }
 for(let i=0;i<4;i++){baseMarkers.push(footMarker(baseGroup,BASE_COLOR));otherMarkers.push(footMarker(otherGroup,VAR_COLOR));}
}
function checkReplay(x,key){if(x.format!=='xgo-lite3-original-stl-mujoco-replay-v1'||x.source?.vendor_commit!==COMMIT||x.experiment?.variation!==key||x.experiment?.seed!==20261106||x.layout?.stride!==162||x.layout?.body_names.length!==17||x.layout.body_names.some((n,i)=>n!==spec.meshes[i].body)||!Array.isArray(x.frames)||x.frames.length!==96)throw Error('Неверная запись '+key);
 for(let i=0;i<x.frames.length;i++){const a=x.frames[i];if(a.length!==162||!a.every(Number.isFinite)||(i>0&&a[0]<=x.frames[i-1][0]))throw Error('Повреждённые кадры '+key);}
 if(Math.abs(x.frames.at(-1)[0]-9.5)>1e-5)throw Error('Длительность не совпадает');return x;
}
async function getReplay(key){const r=await fetch('/hexapod/data/'+files[key],{cache:'no-store'});if(!r.ok)throw Error('Запись отсутствует: '+files[key]);return checkReplay(await r.json(),key);}
function rowAt(replay,time){let lo=0,hi=replay.frames.length-1,a=replay.frames;while(hi-lo>1){let mid=(hi+lo)>>1;if(a[mid][0]<time)lo=mid;else hi=mid;}
 let p=a[lo],q=a[hi],f=Math.max(0,Math.min(1,(time-p[0])/(q[0]-p[0]||1)));return {p,q,f,near:f<.5?p:q};}
function sample(r,i){return r.p[i]+(r.q[i]-r.p[i])*r.f;}
function loc(r){return [sample(r,31),sample(r,32),sample(r,33)];}
function offsetFor(which){return mode==='side'?(which==='base'?-0.28:0.28):0;}
function drawBody(r,meshes,group,markers,which){const off=offsetFor(which);group.position.x=off;
 for(let i=0;i<17;i++){const o=31+i*7,m=meshes[i];m.position.set(sample(r,o),sample(r,o+1),sample(r,o+2));
  qa.set(r.p[o+4],r.p[o+5],r.p[o+6],r.p[o+3]);qb.set(r.q[o+4],r.q[o+5],r.q[o+6],r.q[o+3]);m.quaternion.copy(qa).slerp(qb,r.f).normalize();}
 for(let i=0;i<4;i++){const g=markers[i],n=r.near;g.visible=$('#feet').checked&&n[22+i]>.2;g.position.set(n[150+i*3],n[151+i*3],.003);}
}
function makeTrail(data,group,color){const a=data.frames.map(x=>new THREE.Vector3(x[31],x[32],.003));const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(a),new THREE.LineBasicMaterial({color,transparent:true,opacity:.94}));group.add(line);return line;}
function updateTrails(){if(baseTrail){baseGroup.remove(baseTrail);baseTrail.geometry.dispose();baseTrail.material.dispose();}if(otherTrail){otherGroup.remove(otherTrail);otherTrail.geometry.dispose();otherTrail.material.dispose();}
 baseTrail=makeTrail(baseline,baseGroup,BASE_COLOR);otherTrail=makeTrail(experiment,otherGroup,VAR_COLOR);baseTrail.visible=otherTrail.visible=$('#trails').checked;}
const segmentName=time=>time<1.3?'Стоянка':time<3.1?'Вперёд':time<4.7?'Вбок':time<6.3?'Поворот':time<7.9?'Назад':'Остановка';
const fmtTime=t=>`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;
function setPlaying(v){playing=!!v;$('#play').textContent=playing?'Ⅱ':'▶';$('#play').setAttribute('aria-label',playing?'Пауза':'Играть');}
function showFrame(time){if(!ready||!baseline||!experiment)return;
 const b=rowAt(baseline,time),e=rowAt(experiment,time);drawBody(b,baseMeshes,baseGroup,baseMarkers,'base');drawBody(e,otherMeshes,otherGroup,otherMarkers,'variant');
 const p=loc(b),q=loc(e),dist=Math.hypot(p[0]-q[0],p[1]-q[1],p[2]-q[2]);
 $('#delta').textContent=(dist*1000).toFixed(1)+' мм';$('#segment').textContent=segmentName(time)+' · t '+time.toFixed(2)+' с';
 $('#clock').textContent=fmtTime(time);$('#seek').value=String(time);
 if($('#follow').checked){const now=new THREE.Vector3((p[0]+q[0])/2,.06,-(p[1]+q[1])/2);if(prevPair){const delta=now.clone().sub(prevPair);camera.position.add(delta);orbit.target.add(delta);}prevPair=now;}else prevPair=null;
 if(mode==='overlay'){
  const a=new THREE.Vector3(p[0],p[1],p[2]+.01),c=new THREE.Vector3(q[0],q[1],q[2]+.01);
  referenceLine.geometry.setFromPoints([a,c]);referenceLine.visible=dist>.003;
 }else referenceLine.visible=false;
 updateChart(time);
}
function buildChart(){const a=baseline.frames,b=experiment.frames;
 const X=[...a,...b].map(r=>r[31]-a[0][31]);const Y=[...a,...b].map(r=>r[32]-a[0][32]);
 let xmin=Math.min(...X,0),xmax=Math.max(...X,0),ymin=Math.min(...Y,0),ymax=Math.max(...Y,0);
 const margin=.06;xmin-=margin;xmax+=margin;ymin-=margin;ymax+=margin;
 // Equal physical scale on X and Y; no stretching to make a visual effect.
 const scale=Math.min(375/(xmax-xmin),111/(ymax-ymin));const ox=(430-(xmax-xmin)*scale)/2,oy=(145-(ymax-ymin)*scale)/2;
 chartTransform=row=>[(row[31]-a[0][31]-xmin)*scale+ox,145-((row[32]-a[0][32]-ymin)*scale+oy)];
 const path=rows=>rows.map((r,i)=>{const [x,y]=chartTransform(r);return (i?'L':'M')+x.toFixed(1)+','+y.toFixed(1);}).join(' ');
 $('#chart').innerHTML=`<path d="${path(a)}" fill="none" stroke="#56dcf0" stroke-width="2.5" stroke-linecap="round"/><path d="${path(b)}" fill="none" stroke="#ffbd76" stroke-width="2.5" stroke-linecap="round"/><circle id="base-dot" r="5" fill="#56dcf0"/><circle id="other-dot" r="5" fill="#ffbd76"/><text x="10" y="15" fill="#94afc1" font-size="11">Фактическая траектория корпуса · вид сверху</text>`;
}
function updateChart(time){if(!chartTransform)return;
 const b=rowAt(baseline,time),e=rowAt(experiment,time);
 const bXY=chartTransform({31:sample(b,31),32:sample(b,32)}),eXY=chartTransform({31:sample(e,31),32:sample(e,32)});
 for(const [id,xy] of [['#base-dot',bXY],['#other-dot',eXY]]){$(id).setAttribute('cx',xy[0]);$(id).setAttribute('cy',xy[1]);}
}

function viewMode(w){mode=w;$('#overlay').setAttribute('aria-pressed',String(w==='overlay'));$('#separate').setAttribute('aria-pressed',String(w==='side'));cameraReset();showFrame(t);}
async function change(){const serial=++loadSerial,key=$('#variant').value;setPlaying(false);notice.hidden=false;notify('Загружаем эксперимент '+key+'…');error.hidden=true;
 try{const next=await getReplay(key);if(serial!==loadSerial)return;experiment=next;updateTrails();
  const p=baseline.frames.at(-1),q=experiment.frames.at(-1),d=Math.hypot(q[31]-p[31],q[32]-p[32],q[33]-p[33]);$('#finaldelta').textContent=(d*1000).toFixed(0)+' мм';
  $('#slip').textContent=stats.original_baseline.slip.toFixed(1)+' / '+stats[key].slip.toFixed(1)+'%';
  $('#saturation').textContent=stats.original_baseline.sat.toFixed(1)+' / '+stats[key].sat.toFixed(1)+'%';
  t=0;buildChart();cameraReset();showFrame(t);notice.hidden=true;setPlaying(true);
 }catch(e){if(serial!==loadSerial)return;notice.hidden=true;notify('Ошибка записи: '+(e.message||e),true);}}
async function init(){try{if(!crypto?.subtle)throw Error('Нужен безопасный контекст localhost для проверки SHA-256.');
 await loadSTLs();notice.textContent='Загружаем исходную физику…';baseline=await getReplay('original_baseline');
 referenceLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xffffff,transparent:true,opacity:.8}));world.add(referenceLine);
 ready=true;await change();}catch(e){notice.hidden=true;notify('Не удалось загрузить исходную модель: '+(e.message||e)+'\nВ первый раз необходим интернет для оригинальных 17 STL.',true);setPlaying(false);}}
$('#variant').addEventListener('change',change);$('#play').addEventListener('click',()=>setPlaying(!playing));
$('#seek').addEventListener('input',()=>{t=Number($('#seek').value);showFrame(t);});
$('#overlay').addEventListener('click',()=>viewMode('overlay'));$('#separate').addEventListener('click',()=>viewMode('side'));
$('#top').addEventListener('click',()=>{cameraTop();showFrame(t);});$('#camera').addEventListener('click',()=>{cameraReset();showFrame(t);});
$('#trails').addEventListener('change',()=>{if(baseTrail){baseTrail.visible=otherTrail.visible=$('#trails').checked;}});
$('#feet').addEventListener('change',()=>showFrame(t));$('#follow').addEventListener('change',()=>{prevPair=null;});
window.addEventListener('keydown',e=>{if(e.code==='Space'&&!(e.target instanceof HTMLInputElement)){e.preventDefault();setPlaying(!playing);}});
function loop(timestamp){requestAnimationFrame(loop);if(lastFrame!==null&&playing&&ready&&experiment){const step=Math.min(.1,(timestamp-lastFrame)/1000)*Number($('#speed').value);t=t+step>=9.5?(t+step)%9.5:t+step;showFrame(t);}
 lastFrame=timestamp;orbit.update();renderer.render(scene,camera);}
cameraReset();requestAnimationFrame(loop);init();