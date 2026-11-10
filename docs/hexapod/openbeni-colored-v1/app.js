const REFERENCE_REPLAY='/hexapod/data/openbeni-recovery-v1.json';
// OpenBeni adapter of the existing EN01 measured-record viewer; eight original meshes, no physics in this player.
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
  const cad=await loadVerified(index.rig.cad_asset,index.visual_cad_size,index.visual_cad_sha256,b=>JSON.parse(new TextDecoder().decode(b)));
  if(cad.part_count!==8||cad.parts?.length!==8||cad.source?.commit!==index.source_commit)throw Error('Неверная исходная CAD');
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
  document.documentElement.dataset.cadParts=String(cad.part_count);
}
function resize(){if(!renderer)return;const w=Math.max(1,area.clientWidth),h=Math.max(1,area.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);new ResizeObserver(resize).observe(area);resize();
function frameAt(s){if(s<=frames[0][0])return {a:frames[0],b:frames[0],k:0};if(s>=frames.at(-1)[0])return {a:frames.at(-1),b:frames.at(-1),k:0};let l=0,h=frames.length-1;while(h-l>1){const m=(l+h)>>1;if(frames[m][0]<=s)l=m;else h=m;}return {a:frames[l],b:frames[h],k:(s-frames[l][0])/(frames[h][0]-frames[l][0])};}
function draw(){
  if(!data)return;const {a,b,k}=frameAt(time),lerp=i=>a[i]+(b[i]-a[i])*k;
  groups.forEach((g,j)=>{const i=1+j*7;g.position.set(lerp(i),lerp(i+1),lerp(i+2));qa.set(a[i+4],a[i+5],a[i+6],a[i+3]);qb.set(b[i+4],b[i+5],b[i+6],b[i+3]);g.quaternion.copy(qa).slerp(qb,k);});
  const target=new THREE.Vector3(lerp(1),lerp(3),-lerp(2)),delta=target.clone().sub(follow);if(controls){camera.position.add(delta);controls.target.add(delta);}follow.copy(target);
  $('time').textContent=time.toFixed(1)+' / '+data.duration_s.toFixed(1)+' с';seek.value=String(time);
  const m=data.metric_start;
  $('pose').textContent='Наклон '+a[m].toFixed(1)+'° · скорость '+a[m+1].toFixed(3)+' м/с';
  $('contact').textContent='Колёса: '+a[m+3].toFixed(2)+' / '+a[m+4].toFixed(2)+' Н · корпус '+a[m+2].toFixed(2)+' Н';
  $('joints').textContent=a[m+5]?'Опора на оба колеса':'Переходная поза';
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
    $('verdict').textContent=m.recovered_1s?'Подъём в исходной модели получен':'Подъём не получен';$('verdict').dataset.status='research';
    $('metrics').textContent='Спокойная стойка в конце: '+(m.final_2s_quiet?'да':'нет')+' · наклон '+m.final.tilt_deg.toFixed(1)+'°';
    $('scenario').textContent=clip.label;
    $('result-details').textContent='Начало полной секунды опоры: '+(m.recovery_onset_s===null?'нет':m.recovery_onset_s.toFixed(2)+' с после включения')+'. Максимальное проникновение в пол: '+(m.peak_floor_penetration_m*1000).toFixed(2)+' мм. Это результат упрощённой контактной модели.';
    $('provenance').textContent='OpenBeni 4545b86a · SHA-256 трассы: '+clip.trace_sha256+' · '+next.frame_count+' измеренных кадров.';
    seek.max=String(next.duration_s);
    document.documentElement.dataset.replayClip=clip.id;document.documentElement.dataset.replayReady='true';
    const url=new URL(location.href);url.searchParams.set('clip',clip.id);history.replaceState(null,'',url);
    play.disabled=seek.disabled=false;$('loading').hidden=true;draw();resetCamera();resize();
  }catch(e){if(ticket!==generation)return;$('loading').hidden=true;run(false);error('Не удалось загрузить физическую запись: '+e.message);}
}
play.addEventListener('click',()=>run(!running));seek.addEventListener('input',()=>{time=Number(seek.value);draw();});$('reset-camera').addEventListener('click',resetCamera);
$('case').addEventListener('change',select);
async function init(){try{const r=await fetch('./replays.json');if(!r.ok)throw Error('Каталог HTTP '+r.status);index=validateIndex(await r.json());const ref=index.clips.find(c=>c.id===index.default_clip);await loadVerified(REFERENCE_REPLAY,index.reference.size,index.reference.sha256,b=>decodeRecord(b,ref,index));await robot();const wanted=new URLSearchParams(location.search).get('clip'),clip=index.clips.find(c=>c.id===wanted)||index.clips.find(c=>c.id===index.default_clip);for(const c of index.clips){const option=document.createElement('option');option.value=c.id;option.textContent=c.label;$('case').appendChild(option);}chooseClip(clip);$('case').disabled=false;await select();}catch(e){$('loading').hidden=true;error('Не удалось открыть OpenBeni: '+e.message);run(false);}}
function tick(now){requestAnimationFrame(tick);if(data&&running&&previous!==null){time+=Math.min(.1,(now-previous)/1000)*Number($('speed').value);if(time>data.duration_s)time=0;draw();}previous=now;if(renderer){controls.update();renderer.render(scene,camera);}}
requestAnimationFrame(tick);init();
