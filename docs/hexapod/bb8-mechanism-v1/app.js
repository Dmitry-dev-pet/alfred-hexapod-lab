import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { validatePackage, frameAt, sha256 } from './data.js';

const MODEL_SHA='e4984076ee368406a02e29a01df49059b245566c7c93564a032c8435ae930e80';
const MODEL_URL='/assets/robot-models/bb8-hweigo-'+MODEL_SHA+'.json';
const $=id=>document.getElementById(id),viewer=$('viewer');
let renderer, controls, pack, staticRoot, replayRoot, innerRoot;
let playing=false, time=0, previous=0, view='cutaway';
const nodes=[], geometries=new Map();
const scene=new THREE.Scene();scene.background=new THREE.Color('#101e2b');
const camera=new THREE.PerspectiveCamera(39,1,.001,15);camera.up.set(0,0,1);
const clip=new THREE.Plane(new THREE.Vector3(0,1,0),0);
const Y=new THREE.Vector3(0,1,0);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(8,8),new THREE.MeshStandardMaterial({color:0x142a39,roughness:1}));
floor.position.z=-.001;scene.add(floor);
const grid=new THREE.GridHelper(2,40,0x385064,0x223b4d);grid.rotation.x=Math.PI/2;scene.add(grid);
scene.add(new THREE.HemisphereLight(0xd4edff,0x34404a,2));
const key=new THREE.DirectionalLight(0xffe9cf,3.5);key.position.set(.4,-.5,.9);scene.add(key);
const fill=new THREE.DirectionalLight(0xb5dfff,1.8);fill.position.set(-.5,.4,.3);scene.add(fill);

function category(name){
  if(name==='球壳1.SLDPRT')return 'shell';
  if(/球壳小|头部|舵机|SG90|M1-|牛眼轮/.test(name))return 'head';
  if(/电池|主控/.test(name))return 'power';
  if(/电机|轮胎|轮子|轮轴|联轴器/.test(name))return 'drive';
  return 'frame';
}
function partName(name){
  if(name==='球壳1.SLDPRT')return 'Оболочка шара · 200 мм';
  if(name==='球壳小.SLDPRT')return 'Оболочка головы · 120 мм';
  if(/头部底面/.test(name))return 'Основание головы';
  if(/轮电机/.test(name))return 'Двигатель колеса';
  if(/电机座板/.test(name))return 'Платформа двигателей';
  if(/电机支架/.test(name))return 'Крепление двигателя';
  if(/轮胎/.test(name))return 'Шина колеса';
  if(/轮子/.test(name))return 'Колесо привода';
  if(/轮轴/.test(name))return 'Ось колеса';
  if(/联轴器/.test(name))return 'Муфта вала';
  if(/电池/.test(name))return 'Аккумулятор';
  if(/主控座/.test(name))return 'Крепление платы управления';
  if(/主控板/.test(name))return 'Плата управления';
  if(/SG90/.test(name))return 'Сервопривод SG90';
  if(/牛眼轮/.test(name))return 'Шаровая опора верхнего узла';
  if(/舵机座/.test(name))return 'Площадка сервопривода';
  if(/舵机转/.test(name))return 'Поворотная площадка';
  if(/M1-/.test(name))return 'Деталь верхнего привода';
  if(/铜柱/.test(name))return 'Стойка рамы';
  if(/铜套/.test(name))return 'Втулка или крепление рамы';
  return 'Деталь рамы';
}
function color(name,cat){
  if(cat==='shell')return 0xe2e7e8;
  if(/球壳小|头部底面/.test(name))return 0xeae3cd;
  if(/轮胎/.test(name))return 0x313c46;
  if(/轮子/.test(name))return 0xe59d52;
  if(/电池/.test(name))return 0x4e89be;
  if(/主控/.test(name))return 0x47ad91;
  if(cat==='head')return 0xd6b16b;
  if(cat==='drive')return 0x849cac;
  return 0x9aacb8;
}
function makeNode(item,parent,isReplay){
  const spec=pack.meshes[item.mesh],name=spec.source_file,cat=category(name);
  const material=new THREE.MeshStandardMaterial({color:color(name,cat),roughness:.62,metalness:cat==='frame'?.35:.08,side:THREE.DoubleSide});
  const mesh=new THREE.Mesh(geometries.get(item.mesh),material);
  mesh.position.fromArray(item.position);mesh.quaternion.fromArray(item.quaternion).normalize();
  mesh.userData={name,cat,isReplay,origin:mesh.position.clone()};
  mesh.updateMatrixWorld(true);
  // Explode along each actual part's transformed geometric centre.
  const center=mesh.geometry.boundingSphere.center.clone().applyQuaternion(mesh.quaternion).add(mesh.position);
  mesh.userData.explode=center.length()>.001?center.normalize().multiplyScalar(.075):new THREE.Vector3(0,0,.04);
  parent.add(mesh);nodes.push(mesh);
}
function resetCamera(){camera.position.set(.39,-.52,.34);controls?.target.set(0,0,.125);controls?.update();}
function resize(){const r=viewer.getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/Math.max(r.height,1);camera.updateProjectionMatrix();}
function changeView(){
  if(!pack)return;
  const replay=$('mode').value==='replay',focus=$('focus').value;
  const exploded=!replay&&Number($('explode').value)>0;
  const percent=Math.round(Number($('explode').value)*100);
  $('explode-value').textContent=percent+'%';$('assemble').hidden=!exploded;
  if(!renderer){
    $('fallback-image').src='./'+(!replay&&view==='assembly'?'exterior':'internal')+'.png';
    $('explode').disabled=true;$('focus').disabled=true;$('reset-camera').disabled=true;
    $('selection').textContent=replay?'Показания записи доступны ниже; 3D-воспроизведение недоступно':'Статичный вид исходных деталей · для вращения нужен WebGL';
  }
  staticRoot.visible=!replay;replayRoot.visible=replay;
  $('cad-controls').hidden=replay;$('replay-controls').hidden=!replay;
  $('torque').hidden=!replay;$('focus').hidden=replay;
  $('mode-stamp').textContent=replay?'MuJoCo · диагностическая запись':exploded?'Разнесённый вид · '+percent+'%':'Исходная CAD-сборка';
  $('mode-stamp').dataset.exploded=String(exploded);
  $('counts').textContent=replay?'Голова не участвует в расчёте':'26 деталей · 54 видимых размещения';
  for(const n of nodes){
    const d=n.userData,outer=d.cat==='shell'||/球壳小|头部底面/.test(d.name);
    n.visible=!(outer&&view==='internal');
    n.material.clippingPlanes=outer&&view==='cutaway'?[clip]:[];
    n.material.transparent=outer&&view==='cutaway';
    n.material.opacity=outer&&view==='cutaway'?.33:1;
    n.material.depthWrite=!(outer&&view==='cutaway');
    n.material.emissive.setHex(0);
    if(!replay&&focus!=='all'&&d.cat!==focus){n.material.transparent=true;n.material.opacity=outer?.06:.12;n.material.depthWrite=false;}
    if(!replay&&focus!=='all'&&d.cat===focus)n.material.emissive.setHex(0x493111);
    n.material.needsUpdate=true;
    n.position.copy(d.origin);
    if(!d.isReplay&&d.cat!=='shell')n.position.addScaledVector(d.explode,Number($('explode').value));
  }
  const captions={
    all:'Два колеса внутри шара приводят оболочку в движение. В разрезе видны моторы, колёса и опорный механизм.',
    drive:'Выделены двигатели, колёса, шины и соединения валов. Контакты колёс с оболочкой ещё предстоит воспроизвести в физике.',
    head:'Выделены голова, её основание, сервопривод и верхние опоры. Удержание головы при движении пока не рассчитано.',
    power:'Плата управления и аккумулятор находятся внутри шара. Их расположение показано по исходной CAD-сборке.'
  };
  $('caption').textContent=replay?'Записанный опыт с эквивалентным приводом: 1,5 с разгон, 1,5 с обратный момент, 3 с без момента. Массы и трение приблизительные; голова скрыта.':captions[focus];
  if(exploded)$('caption').textContent='Детали разнесены для осмотра. Нажмите «Собрать», чтобы вернуть их в рабочее положение.';
  for(const b of document.querySelectorAll('[data-view]'))b.setAttribute('aria-pressed',String(b.dataset.view===view&&!(view==='assembly'&&exploded)));
}
function setPlaying(value){playing=value;$('play').textContent=value?'Ⅱ':'▶';$('play').setAttribute('aria-label',value?'Пауза':'Воспроизвести');}
function applyFrame(){
  if(!pack||$('mode').value!=='replay')return;
  const trace=pack.traces[Number($('torque').value)],f=frameAt(trace,time),q=f.qpos;
  replayRoot.position.fromArray(q,0);replayRoot.quaternion.set(q[4],q[5],q[6],q[3]).normalize();
  innerRoot.quaternion.setFromAxisAngle(Y,q[7]);
  $('seek').value=String(f.t);$('time').textContent=f.t.toFixed(2).replace('.',',')+' / 6,00 с';
  $('metric').textContent='x '+(q[0]*100).toFixed(1).replace('.',',')+' см · '+(q[7]*180/Math.PI).toFixed(1).replace('.',',')+'°';
  viewer.dataset.frame=String(Math.round(f.t/.02));
}
function animate(now){
  const delta=previous?Math.min((now-previous)/1000,.1):0;previous=now;
  if(playing&&$('mode').value==='replay'){
    time=Math.min(6,time+delta*Number($('speed').value));applyFrame();if(time>=6)setPlaying(false);
  }
  controls.update();renderer.render(scene,camera);requestAnimationFrame(animate);
}
async function start(){
  try{
    try{
      renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.localClippingEnabled=true;
      viewer.append(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.15;controls.maxDistance=2;resetCamera();
      new ResizeObserver(resize).observe(viewer);resize();requestAnimationFrame(animate);
    }catch(e){
      renderer=null;$('fallback').hidden=false;
      const tick=now=>{const delta=previous?Math.min((now-previous)/1000,.1):0;previous=now;if(playing){time=Math.min(6,time+delta*Number($('speed').value));applyFrame();if(time>=6)setPlaying(false);}requestAnimationFrame(tick);};requestAnimationFrame(tick);
    }
    const response=await fetch(MODEL_URL);if(!response.ok)throw Error('Пакет модели недоступен');
    const bytes=await response.arrayBuffer();if(bytes.byteLength!==3553460||await sha256(bytes)!==MODEL_SHA)throw Error('Контрольная сумма пакета не совпадает');
    pack=validatePackage(JSON.parse(new TextDecoder().decode(bytes)));
    const loader=new STLLoader();let loaded=0;
    for(const [id,spec] of Object.entries(pack.meshes)){
      const raw=Uint8Array.from(atob(spec.stl),c=>c.charCodeAt(0));
      if(raw.byteLength!==spec.size||await sha256(raw)!==spec.sha256)throw Error('Повреждена деталь '+id);
      const geometry=loader.parse(raw.buffer);geometry.scale(.001,.001,.001);geometry.computeBoundingSphere();geometries.set(id,geometry);
      $('loading').textContent='Исходные детали: '+(++loaded)+' / 26';
    }
    staticRoot=new THREE.Group();staticRoot.position.z=.1;scene.add(staticRoot);
    replayRoot=new THREE.Group();scene.add(replayRoot);innerRoot=new THREE.Group();replayRoot.add(innerRoot);
    pack.instances.forEach(i=>makeNode(i,staticRoot,false));
    pack.replay_instances.forEach(i=>makeNode(i,i.group==='inner'?innerRoot:replayRoot,true));
    changeView();$('loading').hidden=true;$('play').disabled=false;viewer.dataset.ready='true';
    if(!renderer)return;
    const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();let down;
    renderer.domElement.addEventListener('pointerdown',e=>down=[e.clientX,e.clientY]);
    renderer.domElement.addEventListener('pointerup',e=>{
      if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>7)return;
      const r=viewer.getBoundingClientRect();pointer.set((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1);
      raycaster.setFromCamera(pointer,camera);
      const replay=$('mode').value==='replay';
      const available=nodes.filter(n=>n.visible&&n.userData.isReplay===replay&&n.material.opacity>.25&&!(view==='cutaway'&&n.userData.cat==='shell'));
      const hit=raycaster.intersectObjects(available).find(h=>!h.object.material.clippingPlanes.length||clip.distanceToPoint(h.point)>=0);
      if(hit)$('selection').textContent=partName(hit.object.userData.name)+' · '+hit.object.userData.name.replace(/\.sldprt$/i,'');
    });
  }catch(e){$('loading').hidden=true;$('error').hidden=false;$('error').textContent=e.message||'Не удалось открыть 3D';console.error(e);}
}
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{view=b.dataset.view;if(view==='assembly')$('explode').value='0';changeView();}));
$('assemble').addEventListener('click',()=>{view='assembly';$('explode').value='0';changeView();document.querySelector('[data-view="assembly"]').focus();});
$('mode').addEventListener('change',()=>{setPlaying(false);time=0;changeView();applyFrame();if(renderer)$('selection').textContent=$('mode').value==='replay'?'Записанные положения оболочки и внутреннего узла':'Нажмите на деталь, чтобы узнать её назначение';});
$('focus').addEventListener('change',changeView);$('explode').addEventListener('input',changeView);
$('torque').addEventListener('change',()=>{time=0;setPlaying(false);applyFrame();});
$('seek').addEventListener('input',()=>{setPlaying(false);time=Number($('seek').value);applyFrame();});
$('play').addEventListener('click',()=>{if(time>=6)time=0;setPlaying(!playing);applyFrame();});
$('reset-camera').addEventListener('click',()=>controls&&resetCamera());
function about(show){$('about').hidden=!show;$('about-toggle').setAttribute('aria-expanded',String(show));}
$('about-toggle').addEventListener('click',()=>about($('about').hidden));$('about-close').addEventListener('click',()=>{about(false);$('about-toggle').focus();});
window.addEventListener('keydown',e=>{if(e.key==='Escape')about(false);if(e.code==='Space'&&e.target===document.body&&$('mode').value==='replay'){e.preventDefault();$('play').click();}});
start();
