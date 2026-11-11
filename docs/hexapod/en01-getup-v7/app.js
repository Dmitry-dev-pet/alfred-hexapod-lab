// Measured nine-body states from the source four-bar adapter; no simulation here.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {validateIndex,decodeRecord,loadVerified} from './record-loader.js';
const $=id=>document.getElementById(id),area=$('viewer'),seek=$('seek'),play=$('play');
const names={stand:'Стойка',push:'Толчок',drive:'Езда',crouch:'Приседание',jump:'Попытка прыжка',fall:'Боковое падение'};
let renderer,controls,index,data,groups=[],frames=[],time=0,running=true,previous=null,generation=0;
const scene=new THREE.Scene();scene.background=new THREE.Color('#122836');
const camera=new THREE.PerspectiveCamera(46,1,.008,40),follow=new THREE.Vector3();
const qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
function error(s){$('replay-error').hidden=false;$('replay-error').textContent=s;}
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.7));renderer.outputColorSpace=THREE.SRGBColorSpace;area.appendChild(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.12;controls.maxDistance=15;}
catch{$('webgl-note').hidden=false;}
scene.add(new THREE.HemisphereLight(0xe0f2ff,0x394d55,2.6));
const light=new THREE.DirectionalLight(0xffffff,2.3);light.position.set(-.65,1.4,.85);scene.add(light);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(40,40),new THREE.MeshStandardMaterial({color:0x243c4c}));ground.rotation.x=-Math.PI/2;scene.add(ground);
const grid=new THREE.GridHelper(40,400,0x5a7682,0x38505e);grid.position.y=.0005;scene.add(grid);
const zUp=new THREE.Group();zUp.rotation.x=-Math.PI/2;scene.add(zUp);
function decoded(s,Type){const a=Uint8Array.from(atob(s),c=>c.charCodeAt(0));if(a.length%4)throw Error('Повреждена геометрия');return new Type(a.buffer);}
async function robot(){
  const cad=await loadVerified('/'+index.rig.cad_asset,index.cad_size,index.cad_sha256,b=>JSON.parse(new TextDecoder().decode(b)));
  if(cad.part_count!==73||cad.parts?.length!==73||cad.source?.git_blob_sha1!==index.step_git_blob_sha1)throw Error('Неверная исходная CAD');
  groups=index.rig.body_names.map(()=>{const g=new THREE.Group();zUp.add(g);return g;});
  for(const p of cad.parts){
    const j=index.rig.body_names.indexOf(index.rig.part_body_map[String(p.id)]);if(j<0)throw Error('Неизвестное физическое звено');
    const v=decoded(p.vertices_f32le,Float32Array),ix=decoded(p.indices_u32le,Uint32Array);
    if(v.length!==p.vertex_count*3||ix.length!==p.triangles*3||!v.every(Number.isFinite)||ix.some(i=>i>=p.vertex_count))throw Error('Повреждена CAD');
    const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(v,3));geo.setIndex(new THREE.BufferAttribute(ix,1));
    geo.translate(...index.rig.rest_body_positions[j].map(v=>-v));geo.computeVertexNormals();
    const color=new THREE.Color().setRGB(...p.color.slice(0,3),THREE.SRGBColorSpace);
    const mesh=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color,roughness:.48,metalness:.2,side:THREE.DoubleSide}));mesh.name=p.name;groups[j].add(mesh);
  }
  document.documentElement.dataset.cadParts='73';
}
function resize(){if(!renderer)return;const w=Math.max(1,area.clientWidth),h=Math.max(1,area.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);new ResizeObserver(resize).observe(area);resize();
function frameAt(s){if(s<=frames[0][0])return {a:frames[0],b:frames[0],k:0};if(s>=frames.at(-1)[0])return {a:frames.at(-1),b:frames.at(-1),k:0};let l=0,h=frames.length-1;while(h-l>1){const m=(l+h)>>1;if(frames[m][0]<=s)l=m;else h=m;}return {a:frames[l],b:frames[h],k:(s-frames[l][0])/(frames[h][0]-frames[l][0])};}
function draw(){
  if(!data)return;const {a,b,k}=frameAt(time),lerp=i=>a[i]+(b[i]-a[i])*k;
  groups.forEach((g,j)=>{const i=1+j*7;g.position.set(lerp(i),lerp(i+1),lerp(i+2));qa.set(a[i+4],a[i+5],a[i+6],a[i+3]);qb.set(b[i+4],b[i+5],b[i+6],b[i+3]);g.quaternion.copy(qa).slerp(qb,k);});
  const target=new THREE.Vector3(lerp(1),lerp(3),-lerp(2)),delta=target.clone().sub(follow);if(controls){camera.position.add(delta);controls.target.add(delta);}follow.copy(target);
  $('time').textContent=time.toFixed(1)+' / 6,0 с';seek.value=String(time);
  $('pose').textContent='Наклон '+(Math.acos(Math.max(-1,Math.min(1,1-2*(a[5]*a[5]+a[6]*a[6]))))*180/Math.PI).toFixed(1)+'° · x '+lerp(1).toFixed(3)+' м';
  $('contact').textContent=(a[64]?'Опора есть':'Нет опоры')+(a[65]?' · звено на полу':'')+' · '+a[66].toFixed(2)+' Н';
  $('joints').textContent='Замыкание '+(lerp(69)*1000).toFixed(3)+' мм';
}
function resetCamera(){if(!data||!controls)return;const {a}=frameAt(time),t=new THREE.Vector3(a[1],a[3],-a[2]);controls.target.copy(t);camera.position.copy(t).add(new THREE.Vector3(.25,.20,.33));follow.copy(t);controls.update();}
function run(v){running=v;play.textContent=v?'Ⅱ':'▶';play.setAttribute('aria-label',v?'Пауза':'Воспроизвести');}
function chooseClip(c){$('case').value=c.id;}
function selectedClip(){return index.clips.find(c=>c.id===$('case').value);}
async function select(){
  const ticket=++generation,clip=selectedClip();data=null;document.documentElement.dataset.replayReady='false';play.disabled=seek.disabled=true;$('replay-error').hidden=true;$('loading').hidden=false;
  try{
    const next=await loadVerified(clip.url,clip.size,clip.sha256,b=>decodeRecord(b,clip,index));if(ticket!==generation)return;
    data=next;frames=next.frames;time=0;previous=null;
    const m=clip.metrics;
    $('verdict').textContent=clip.getup_pass?'Устойчивый подъём в модели':'Устойчивого подъёма нет';$('verdict').dataset.status=clip.status;
    $('metrics').textContent='В конце: '+m.final_stable_s.toFixed(2)+' с стойки · наклон '+m.final_tilt_deg.toFixed(1)+'°';
    $('scenario').textContent=clip.label;
    $('result-details').textContent='Проверка требует двух секунд опоры на оба колеса с наклоном до 15°, малой скоростью и без опоры корпусом. Максимальная непрерывная стойка: '+m.longest_stable_s.toFixed(3)+' с. Наклон перед командой: '+m.tilt_at_command_deg.toFixed(1)+'°. Контакты и суставы: '+(clip.geometry_ok?'проверки пройдены':'геометрия не прошла проверку')+'.';
    $('provenance').textContent='Исходная трасса SHA-256: '+clip.trace_sha256+' · физическая запись, 300 кадров.';
    document.documentElement.dataset.replayClip=clip.id;document.documentElement.dataset.replayReady='true';
    const url=new URL(location.href);url.searchParams.set('clip',clip.id);history.replaceState(null,'',url);
    play.disabled=seek.disabled=false;$('loading').hidden=true;draw();resetCamera();resize();
  }catch(e){if(ticket!==generation)return;$('loading').hidden=true;run(false);error('Не удалось загрузить физическую запись: '+e.message);}
}
play.addEventListener('click',()=>run(!running));seek.addEventListener('input',()=>{time=Number(seek.value);draw();});$('reset-camera').addEventListener('click',resetCamera);
$('case').addEventListener('change',select);
async function init(){try{const r=await fetch('./replays.json');if(!r.ok)throw Error('Каталог HTTP '+r.status);index=validateIndex(await r.json());await robot();const wanted=new URLSearchParams(location.search).get('clip'),clip=index.clips.find(c=>c.id===wanted)||index.clips.find(c=>c.id===index.default_clip);for(const c of index.clips){const option=document.createElement('option');option.value=c.id;option.textContent=c.label;$('case').appendChild(option);}chooseClip(clip);$('case').disabled=false;await select();}catch(e){$('loading').hidden=true;error('Не удалось открыть EN01: '+e.message);run(false);}}
function tick(now){requestAnimationFrame(tick);if(data&&running&&previous!==null){time+=Math.min(.1,(now-previous)/1000)*Number($('speed').value);if(time>6)time=0;draw();}previous=now;if(renderer){controls.update();renderer.render(scene,camera);}}
requestAnimationFrame(tick);init();
