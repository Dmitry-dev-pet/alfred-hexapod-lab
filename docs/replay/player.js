import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {loadReplay,sampleAt} from './format.js';
import {readCatalog,libraryEntries,fetchRecording,readBoundedResponse} from './catalog.js';
import {readRobotLibrary,robotFamily} from '../robots/library.js';
import {createLibraryView} from './library-view.js';
const $=id=>document.getElementById(id), area=$('viewer');
let renderer,controls,loaded,clip,time=0,playing=false,last=null,generation=0,groups=[],environs=[];
let catalogEntries=[],catalogGeneration=0;
const scene=new THREE.Scene(),world=new THREE.Group(),camera=new THREE.PerspectiveCamera(45,1,.001,1000);
scene.add(world);const follow=new THREE.Vector3(),qa=new THREE.Quaternion(),qb=new THREE.Quaternion();
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.4));renderer.outputColorSpace=THREE.SRGBColorSpace;area.append(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;}catch{$('webgl').hidden=false;}
function dispose(){world.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});world.clear();for(const o of [...scene.children])if(o.isLight)scene.remove(o);groups=[];environs=[];}
function surface(s){const g=s.kind==='plane'?new THREE.PlaneGeometry(s.size[0],s.size[1]):new THREE.BoxGeometry(...s.size),m=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:s.color,roughness:.9,side:THREE.DoubleSide}));m.position.set(...s.position);if(s.kind==='plane'&&loaded.doc.scene.up_axis==='Y')m.rotation.x=-Math.PI/2;return m;}
function installModel(){dispose();const s=loaded.doc.scene;scene.background=new THREE.Color(s.background);world.rotation.x=s.up_axis==='Z'?-Math.PI/2:0;
 for(const l of s.lights){const light=l.kind==='hemisphere'?new THREE.HemisphereLight(l.color,l.ground,l.intensity):new THREE.DirectionalLight(l.color,l.intensity);light.position.set(...l.position);scene.add(light);}
 groups=loaded.doc.nodes.map(n=>{const g=new THREE.Group();g.name=n.id;world.add(g);return g;});
 if(renderer)for(const m of loaded.meshes){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(m.positions,3));g.setIndex(new THREE.BufferAttribute(m.indices,1));g.scale(m.scale,m.scale,m.scale);g.translate(...m.translation);g.computeVertexNormals();groups[m.node].add(new THREE.Mesh(g,new THREE.MeshStandardMaterial({...m.material,side:THREE.DoubleSide})));}
 document.documentElement.dataset.meshCount=String(loaded.meshes.length);
}
function setPlaying(v){playing=v;last=null;$('play').textContent=v?'Ⅱ':'▶';$('play').setAttribute('aria-label',v?'Пауза':'Воспроизвести');}
const stamp=t=>Math.floor(t/60)+':'+String(Math.floor(t%60)).padStart(2,'0');
function draw(){if(!clip)return;const f=clip.frames,s=clip.frame_stride,{a,b,k}=sampleAt(clip,time),ao=a*s,bo=b*s;
 for(let j=0;j<groups.length;j++){const i=1+7*j,lerp=n=>f[ao+n]+(f[bo+n]-f[ao+n])*k;groups[j].position.set(lerp(i),lerp(i+1),lerp(i+2));qa.set(f[ao+i+4],f[ao+i+5],f[ao+i+6],f[ao+i+3]);qb.set(f[bo+i+4],f[bo+i+5],f[bo+i+6],f[bo+i+3]);groups[j].quaternion.copy(qa).slerp(qb,k);}
 const target=world.localToWorld(groups[loaded.doc.scene.camera.follow_node].position.clone());
 if(controls){const delta=target.clone().sub(follow);camera.position.add(delta);controls.target.add(delta);}follow.copy(target);
 $('seek').value=String(time);$('time').textContent=stamp(time)+' / '+stamp(clip.duration_s);
 if(clip.contacts){const n=clip.contacts.length/clip.frame_count,values=clip.contacts.subarray(a*n,(a+1)*n);$('contacts').textContent='Нагруженных опор: '+values.filter(v=>v>.1).length+' / '+n+' · '+values.reduce((x,y)=>x+y,0).toFixed(1)+' Н';}else $('contacts').textContent='';
 document.documentElement.dataset.frame=String(a);document.documentElement.dataset.time=String(time);
}
function resetCamera(){if(!clip||!controls)return;draw();camera.position.copy(follow).add(new THREE.Vector3(...loaded.doc.scene.camera.offset));controls.target.copy(follow);controls.minDistance=.01;controls.maxDistance=100;controls.update();}
function select(id){clip=loaded.clips.find(c=>c.id===id);time=0;last=null;for(const m of environs){world.remove(m);m.geometry.dispose();m.material.dispose();}environs=clip.environment.map(surface);world.add(...environs);$('seek').max=String(clip.duration_s);$('result').textContent=clip.result||'';$('metrics').textContent=clip.metrics?JSON.stringify(clip.metrics,null,2):'';$('provenance').textContent=clip.frame_count+' измеренных кадров'+(clip.source_trace_sha256?' · SHA-256 исходной трассы: '+clip.source_trace_sha256:'');document.documentElement.dataset.clip=clip.id;draw();resetCamera();setPlaying(true);}
function resize(){if(!renderer)return;const w=Math.max(1,area.clientWidth),h=Math.max(1,area.clientHeight);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
new ResizeObserver(resize).observe(area);
function setView(view) {
  document.documentElement.dataset.view = view;
  $('empty').hidden = view !== 'library';
  $('library').hidden = view === 'library';
  resize();
}
function enableTransport() {
  for (const id of ['clip', 'seek', 'play', 'reset']) $(id).disabled = !clip;
}
function showFailure(message, ticket) {
  if (ticket !== generation) return;
  $('loading').hidden = true;
  $('error').hidden = false;
  $('error').textContent = message;
  enableTransport();
}
function setAddress(key, value) {
  const url = new URL(location.href);
  url.searchParams.delete('recording');
  url.searchParams.delete('file');
  if (key) url.searchParams.set(key, value);
  if (url.href !== location.href) history.pushState(null, '', url);
}
function begin() {
  const ticket = ++generation;
  setPlaying(false);
  setView('player');
  $('error').hidden = true;
  $('loading').hidden = false;
  $('loading').textContent = 'Открываем запись…';
  for (const id of ['clip', 'seek', 'play', 'reset']) $(id).disabled = true;
  return ticket;
}
async function openBytes(bytes, name, ticket) {
  if (ticket !== generation) return false;
  try {
    const parsed = await loadReplay(bytes, status => {
      if (ticket === generation) $('loading').textContent = status;
    });
    if (ticket !== generation) return false;
    loaded = parsed;
    clip = null;
    installModel();
    $('clip').replaceChildren(...loaded.clips.map(c => {
      const option = document.createElement('option');
      option.value = c.id; option.textContent = c.title;
      return option;
    }));
    $('clip').value = loaded.doc.default_clip;
    $('title').textContent = loaded.doc.metadata.title;
    $('description').textContent = loaded.doc.metadata.description || name;
    $('loading').hidden = true;
    select(loaded.doc.default_clip);
    enableTransport();
    resize();
    document.documentElement.dataset.replayReady = 'true';
    document.documentElement.dataset.source = name;
    $('title').tabIndex = -1;
    $('title').focus({preventScroll: true});
    return true;
  } catch (error) {
    showFailure('Не удалось открыть запись: ' + error.message, ticket);
    return false;
  }
}
async function openFile(file) {
  if (!file) return;
  const ticket = begin();
  try {
    if (file.size > 100 * 1024 * 1024) throw Error('Файл больше 100 МБ');
    if (await openBytes(await file.arrayBuffer(), file.name, ticket)) {
      setAddress();
      libraryView.select(null);
    }
  } catch (error) { showFailure('Не удалось прочитать файл: ' + error.message, ticket); }
}
async function openURL(raw, entry, updateHistory = true) {
  const ticket = begin();
  try {
    const url = new URL(raw, location.href);
    if (!['http:', 'https:'].includes(url.protocol)) throw Error('Нужна HTTP-ссылка на файл');
    const bytes = entry ? await fetchRecording(entry) : await readBoundedResponse(await fetch(url, {
      credentials: 'same-origin', redirect: 'error', signal: AbortSignal.timeout(60000)
    }));
    if (await openBytes(bytes, entry?.title || url.pathname.split('/').pop(), ticket)) {
      libraryView.select(entry?.id);
      if (updateHistory) setAddress(entry ? 'recording' : 'file', entry?.id || url.href);
    }
  } catch (error) { showFailure('Не удалось загрузить запись: ' + error.message, ticket); }
}
async function initCatalog() {
  const ticket = ++catalogGeneration;
  let doc;
  try { doc = await readRobotLibrary(); }
  catch {
    const fallback = await readCatalog();
    doc = {
      entries: libraryEntries(fallback).map(e => ({
        ...e, kind: 'recording', robot: robotFamily(e.id), availability: 'published'
      })),
      source: 'replay-only fallback'
    };
  }
  if (ticket !== catalogGeneration) return;
  catalogEntries = doc.entries;
  libraryView.update(doc);
}
function showLibrary(updateHistory = true) {
  ++generation; // A previous download/decoder cannot replace the library after returning.
  setPlaying(false);
  $('loading').hidden = true;
  $('error').hidden = true;
  enableTransport();
  setView('library');
  if (updateHistory) setAddress();
}
async function openLocation() {
  const params = new URLSearchParams(location.search);
  if (params.get('file')) return openURL(params.get('file'), null, false);
  const selected = params.get('recording');
  if (!selected) return showLibrary(false);
  const entry = catalogEntries.find(e => e.id === selected && e.kind === 'recording' && e.availability === 'published');
  if (entry) return openURL(entry.recording.url, entry, false);
  showLibrary(false);
  libraryView.showError(Error('Запись не найдена в библиотеке'));
}
const libraryView = createLibraryView({
  document, onOpen: entry => openURL(entry.recording.url, entry), onRefresh: initCatalog
});
window.addEventListener('dragover', event => event.preventDefault());
window.addEventListener('drop', event => {
  event.preventDefault();
  openFile(event.dataTransfer?.files?.[0]);
});
$('choose').onclick = $('import-file').onclick = $('another').onclick = () => $('file').click();
$('file').onchange = () => {
  const file = $('file').files[0];
  $('file').value = '';
  openFile(file);
};
$('library').onclick = () => { showLibrary(); $('search').focus({preventScroll: true}); };
$('show-url').onclick = () => { $('url-form').hidden = false; $('url').focus(); };
$('url-form').onsubmit = event => { event.preventDefault(); openURL($('url').value); };
$('play').onclick = () => setPlaying(!playing);
$('clip').onchange = () => select($('clip').value);
$('seek').oninput = () => { time = Number($('seek').value); last = null; draw(); };
$('reset').onclick = resetCamera;
window.addEventListener('robot-recording-catalog', () => initCatalog().catch(libraryView.showError));
window.addEventListener('popstate', () => openLocation());
const initialFile = new URLSearchParams(location.search).get('file');
if (initialFile) {
  openURL(initialFile, null, false);
  initCatalog().catch(libraryView.showError);
} else {
  initCatalog().then(openLocation).catch(libraryView.showError);
}
function tick(now) {
  requestAnimationFrame(tick);
  if (clip && playing && last !== null) {
    time += Math.min(.1, (now - last) / 1000) * Number($('speed').value);
    if (time > clip.duration_s) time = 0;
    draw();
  }
  last = now;
  if (renderer && clip && document.documentElement.dataset.view === 'player') {
    controls.update(); renderer.render(scene, camera);
  }
}
requestAnimationFrame(tick);
resize();
