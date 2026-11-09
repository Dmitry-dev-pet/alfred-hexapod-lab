// Adapter for existing player shell. Draws measured MuJoCo body poses only.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id),area=$('viewer'),seek=$('seek'),play=$('play');
const sourceUrl='/hexapod/data/en01-torque-template.json';
let renderer,controls,data,frames=[],groups=[],time=0,running=true,previous=null;
const scene=new THREE.Scene();scene.background=new THREE.Color('#122836');
const camera=new THREE.PerspectiveCamera(46,1,.008,30),follow=new THREE.Vector3();
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
function error(s){$('replay-error').hidden=false;$('replay-error').textContent=s;}
try{
  renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.7));renderer.outputColorSpace=THREE.SRGBColorSpace;
  area.appendChild(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);
  controls.enableDamping=true;controls.minDistance=.12;controls.maxDistance=12;
}catch(e){error('WebGL недоступен. Метрики физической записи доступны; 3D-проверка требует другого браузера.');}
scene.add(new THREE.HemisphereLight(0xe0f2ff,0x394d55,2.6));
const light=new THREE.DirectionalLight(0xffffff,2.3);light.position.set(-.65,1.4,.85);scene.add(light);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(30,30),new THREE.MeshStandardMaterial({color:0x243c4c}));
ground.rotation.x=-Math.PI/2;scene.add(ground);
const grid=new THREE.GridHelper(30,300,0x5a7682,0x38505e);grid.position.y=.0005;scene.add(grid);
const zUp=new THREE.Group();zUp.rotation.x=-Math.PI/2;scene.add(zUp);
function decoded(s,Type){const a=Uint8Array.from(atob(s),c=>c.charCodeAt(0));if(a.length%4)throw Error('Повреждена геометрия');return new Type(a.buffer);}
async function robot(){
  const r=await fetch('/'+data.cad_asset);if(!r.ok)throw Error('CAD HTTP '+r.status);
  const cad=await r.json();
  if(cad.part_count!==73||cad.parts?.length!==73||cad.source?.git_blob_sha1!==data.source.step_git_blob_sha1)throw Error('Неверный исходный CAD');
  groups=data.body_names.map(()=>{const g=new THREE.Group();zUp.add(g);return g;});
  for(const p of cad.parts){
    const index=data.body_names.indexOf(data.part_body_map[String(p.id)]);if(index<0)throw Error('Нет физического тела детали');
    const v=decoded(p.vertices_f32le,Float32Array),ix=decoded(p.indices_u32le,Uint32Array);
    if(v.length!==p.vertex_count*3||ix.length!==p.triangles*3||!v.every(Number.isFinite)||ix.some(i=>i>=p.vertex_count))throw Error('Повреждён CAD');
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(v,3));geometry.setIndex(new THREE.BufferAttribute(ix,1));
    const rest=data.rest_body_positions[index];geometry.translate(-rest[0],-rest[1],-rest[2]);geometry.computeVertexNormals();
    const color=new THREE.Color().setRGB(...p.color.slice(0,3),THREE.SRGBColorSpace);
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,roughness:.48,metalness:.2,side:THREE.DoubleSide}));
    mesh.name=p.name;groups[index].add(mesh);
  }
  document.documentElement.dataset.cadParts='73';
}
function resize(){if(!renderer)return;const w=Math.max(1,area.clientWidth),h=Math.max(1,area.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();
function frameAt(s){
  if(s<=frames[0][0])return {a:frames[0],b:frames[0],k:0};if(s>=frames.at(-1)[0])return {a:frames.at(-1),b:frames.at(-1),k:0};
  let l=0,h=frames.length-1;while(h-l>1){const m=(l+h)>>1;if(frames[m][0]<=s)l=m;else h=m;}
  return {a:frames[l],b:frames[h],k:(s-frames[l][0])/(frames[h][0]-frames[l][0])};
}
function draw(){
  if(!data)return;const {a,b,k}=frameAt(time),lerp=i=>a[i]+(b[i]-a[i])*k;
  groups.forEach((g,j)=>{const i=1+j*7;g.position.set(lerp(i),lerp(i+1),lerp(i+2));qa.set(a[i+4],a[i+5],a[i+6],a[i+3]);qb.set(b[i+4],b[i+5],b[i+6],b[i+3]);g.quaternion.copy(qa).slerp(qb,k);});
  const target=new THREE.Vector3(lerp(1),lerp(3),-lerp(2)),delta=target.clone().sub(follow);
  if(controls){camera.position.add(delta);controls.target.add(delta);}follow.copy(target);
  const t=1+data.body_names.length*7;
  $('time').textContent=time.toFixed(1)+' / 6,0 с';seek.value=String(time);
  $('pose').textContent='Наклон '+(lerp(t+3)*180/Math.PI).toFixed(1)+'° · x '+lerp(1).toFixed(3)+' м';
  $('contact').textContent=(a[t]?'Опора есть':'Нет опоры')+(a[t+1]?' · контакт звена с полом':'')+' · '+a[t+2].toFixed(2)+' Н';
  $('joints').textContent='Ошибка замыкания '+(lerp(t+5)*1000).toFixed(3)+' мм';
}
function resetCamera(){if(!data||!controls)return;const {a}=frameAt(time),t=new THREE.Vector3(a[1],a[3],-a[2]);controls.target.copy(t);camera.position.copy(t).add(new THREE.Vector3(.25,.20,.33));follow.copy(t);controls.update();}
function run(v){running=v;play.textContent=v?'Ⅱ':'▶';play.setAttribute('aria-label',v?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>run(!running));seek.addEventListener('input',()=>{time=Number(seek.value);draw();});$('reset-camera').addEventListener('click',resetCamera);
async function init(){
  try{
    const r=await fetch(sourceUrl);if(!r.ok)throw Error('HTTP '+r.status);const next=await r.json();
    if(next.format!=='navbot-en01-fourbar-physical-replay-v2'||next.frame_count!==300||next.frames?.length!==300||next.body_names?.length!==9||next.sample_stride!==70||!['research','failed'].includes(next.status))throw Error('Неверная запись');
    let previous=-1;for(const f of next.frames){if(f.length!==70||!f.every(Number.isFinite)||f[0]<=previous)throw Error('Повреждён кадр');previous=f[0];}
    data=next;frames=data.frames;
    const r=data.result;
    $('verdict').textContent=r.getup_pass?'Подъём в модели':'Устойчивого подъёма нет';
    $('scenario').textContent=r.orientation+' · '+r.strategy;
    $('mode').textContent='Устойчивая стойка в конце: '+r.final_stable_s.toFixed(2)+' с · итоговый наклон '+r.final_tilt_deg.toFixed(1)+'°';
    await robot();draw();resetCamera();document.documentElement.dataset.replayReady='true';
  }catch(e){error('Не удалось загрузить физическую запись: '+e.message);run(false);}
}
function tick(now){requestAnimationFrame(tick);if(data&&running&&previous!==null){time+=Math.min(.1,(now-previous)/1000)*Number($('speed').value);if(time>6)time=0;draw();}previous=now;if(renderer){controls.update();renderer.render(scene,camera);}}
requestAnimationFrame(tick);init();
