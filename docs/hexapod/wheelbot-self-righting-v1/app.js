import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
// Player only replays measured MuJoCo mj_step states. No dynamics are synthesized on the tablet.
const el=id=>document.getElementById(id);
const area=el('viewer'),play=el('play'),seek=el('seek'),speed=el('speed'),message=el('wb-error');
const scene=new THREE.Scene();scene.background=new THREE.Color('#10212c');
const camera=new THREE.PerspectiveCamera(43,1,.005,15);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.7));
renderer.outputColorSpace=THREE.SRGBColorSpace;area.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.08;orbit.minDistance=.16;orbit.maxDistance=2.7;
scene.add(new THREE.HemisphereLight(0xdaefff,0x394d5a,2.8));
const sunlight=new THREE.DirectionalLight(0xffffff,2.4);sunlight.position.set(-.6,1.2,1.5);scene.add(sunlight);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(7,7),new THREE.MeshStandardMaterial({color:0x203746,roughness:.9}));ground.rotation.x=-Math.PI/2;scene.add(ground);
const grid=new THREE.GridHelper(5,50,0x537489,0x314b5a);grid.position.y=.0005;scene.add(grid);
const axes=new THREE.AxesHelper(.06);axes.position.set(-.19,.005,-.18);scene.add(axes);
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world); // MuJoCo z-up to Three.js y-up
const body=new THREE.Group();world.add(body);
const mat=(c,metal=.2)=>new THREE.MeshStandardMaterial({color:c,roughness:.48,metalness:metal});
const metal=mat(0xb6c5d1,.65),dark=mat(0x192530,.15),gold=mat(0xdcae52,.5),blue=mat(0x2385b0,.2),tire=mat(0x14171b,.14);
const shape=(parent,geometry,material,x=0,y=0,z=0)=>{const o=new THREE.Mesh(geometry,material);o.position.set(x,y,z);parent.add(o);return o;};
shape(body,new THREE.BoxGeometry(.086,.064,.104),metal);
shape(body,new THREE.BoxGeometry(.062,.003,.068),dark,0,-.034,.004);
shape(body,new THREE.BoxGeometry(.027,.003,.047),blue,.010,-.036,.006);
for(const x of [-.034,.034])shape(body,new THREE.CylinderGeometry(.003,.003,.12,9),gold,x,0,0).rotation.x=Math.PI/2;
const drive=new THREE.Group();drive.position.z=-.065;body.add(drive);
shape(drive,new THREE.CylinderGeometry(.032,.032,.016,48),tire);
shape(drive,new THREE.CylinderGeometry(.011,.011,.018,28),gold);
for(let i=0;i<6;i++){const spoke=shape(drive,new THREE.BoxGeometry(.042,.017,.003),metal);spoke.rotation.y=i*Math.PI/6;}
const reaction=new THREE.Group();reaction.position.z=.065;body.add(reaction);
const flywheel=shape(reaction,new THREE.CylinderGeometry(.032,.032,.016,48),dark);flywheel.rotation.z=Math.PI/2;
const flyrim=shape(reaction,new THREE.CylinderGeometry(.026,.026,.017,40),gold);flyrim.rotation.z=Math.PI/2;
const marker=shape(reaction,new THREE.BoxGeometry(.002,.046,.003),blue);
let dataset=null,frames=[],t=0,running=true,last=null,previous=new THREE.Vector3();
const qq1=new THREE.Quaternion(),qq2=new THREE.Quaternion();
const fmt=s=>Math.floor(s/60)+':'+String(Math.floor(s%60)).padStart(2,'0');
function resize(){let w=Math.max(area.clientWidth,1),h=Math.max(area.clientHeight,1);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false)}
addEventListener('resize',resize);resize();
function loadState(s){if(!Array.isArray(s)||s.length!==12||!s.every(Number.isFinite))throw Error('Повреждён кадр MuJoCo');}
function frameAt(seconds){let lo=0,hi=frames.length-1;while(hi-lo>1){const m=(lo+hi)>>1;if(frames[m][0]<=seconds)lo=m;else hi=m;}
  const a=frames[lo],b=frames[hi],k=Math.max(0,Math.min(1,(seconds-a[0])/(b[0]-a[0]||1)));
  return {a,b,k,lerp:i=>a[i]+(b[i]-a[i])*k};}
function draw(seconds){if(!dataset)return;const {a,b,k,lerp}=frameAt(seconds);
  body.position.set(lerp(1),lerp(2),lerp(3));
  qq1.set(a[5],a[6],a[7],a[4]);qq2.set(b[5],b[6],b[7],b[4]);body.quaternion.copy(qq1.slerp(qq2,k));
  drive.rotation.y=lerp(8);reaction.rotation.x=lerp(9);
  const pos=new THREE.Vector3(lerp(1),lerp(3),-lerp(2)),shift=pos.clone().sub(previous);
  camera.position.add(shift);orbit.target.add(shift);previous.copy(pos);
  const quaternion=body.quaternion;const rpy=new THREE.Euler().setFromQuaternion(quaternion,'XYZ');
  const roll=rpy.x*180/Math.PI;
  el('state').textContent=seconds<1?'Лежит на боку':seconds<4.5?'Разгон реакционного колеса':seconds<5.4?'Подъём и перехват':'Балансировка (приближённая модель)';
  el('pose').textContent=`Крен: ${roll.toFixed(1)}° · высота: ${(lerp(3)*100).toFixed(1)} см · время: ${fmt(seconds)} / ${fmt(dataset.duration_s)}`;
  el('motors').textContent=`Реакционное колесо: ${lerp(10).toFixed(0)} рад/с · момент: ${lerp(11).toFixed(3)} Н·м`;
  seek.value=String(seconds);
}
function resetCamera(){if(!dataset)return;let f=frameAt(t).lerp;let pos=new THREE.Vector3(f(1),f(3),-f(2));orbit.target.copy(pos);camera.position.copy(pos).add(new THREE.Vector3(.28,.19,.36));previous.copy(pos);orbit.update();}
function playState(next){running=next;play.textContent=running?'Ⅱ':'▶';play.setAttribute('aria-label',running?'Пауза':'Воспроизвести');}
play.addEventListener('click',()=>playState(!running));
seek.addEventListener('input',()=>{t=Number(seek.value);draw(t)});
el('reset-camera').addEventListener('click',resetCamera);
async function init(){try{const r=await fetch('/hexapod/data/wheelbot-self-righting-v1.json',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
  dataset=await r.json();if(dataset.format!=='mini-wheelbot-mujoco-physical-replay-v1'||dataset.frame_count!==dataset.frames.length||dataset.sample_stride!==12||dataset.frame_count<50)throw Error('Не соответствует протоколу записи');
  frames=dataset.frames;for(let i=0;i<frames.length;i++){loadState(frames[i]);if(i>0&&!(frames[i][0]>frames[i-1][0]))throw Error('Неверное время кадров')}
  seek.max=String(dataset.duration_s);seek.step='.005';draw(0);resetCamera();
}catch(e){message.hidden=false;message.textContent='Не удалось загрузить физическую запись: '+String(e.message||e);playState(false);}}
function tick(now){requestAnimationFrame(tick);if(running&&dataset&&last!==null){t+=Math.max(0,Math.min(.1,(now-last)/1000))*Number(speed.value||1);if(t>dataset.duration_s)t=0;draw(t);}last=now;orbit.update();renderer.render(scene,camera)}
requestAnimationFrame(tick);init();