/* Portrait-first Hexapod player shell. The physics replay remains in app.js. */
(() => {
  'use strict';
  const layout=document.querySelector('.layout'), stage=document.querySelector('.stage');
  const nativePlay=document.getElementById('play'), nativeSeek=document.getElementById('seek');
  const nativeSpeed=document.getElementById('speed');
  if(!layout||!stage||!nativePlay||!nativeSeek||document.getElementById('hx-shell'))return;
  const fallback=[
    {id:'tripod-60',title:'Tripod',detail:'Обычная походка · 60 с',duration_s:60,href:'/hexapod/'},
    {id:'claws-10',title:'Клешни',detail:'Поднятые передние ноги · 10 с',duration_s:10,href:'/hexapod/claws/'},
    {id:'six-to-four',title:'Переход 6 → 4',detail:'Подготовка четырёхопорной стойки · 7 с',duration_s:7,href:'/hexapod/six-to-four/'}
  ];
  const safeHref=(p)=>typeof p==='string'&&(/\/hexapod\/(?:[a-z0-9-]+\/)?$/.test(p)&&p.startsWith('/hexapod/')||/^\/replay\/\?recording=[a-z0-9-]{1,64}$/.test(p));
  const path=location.pathname;
  // The root address ALWAYS opens the established bottom-sheet library.
  // Never redirect the root back to a saved replay: standalone comparison pages
  // use /hexapod/ as their library link. Previously that caused a redirect loop.
  const libraryView=path==='/hexapod/';
  let entries=fallback,opened=false,visible=true,hideTimer,toastTimer;
  const sourceLabel=location.pathname.startsWith('/hexapod/cheetah-idto-')?'DRAKE · MPC · REPLAY':'MUJOCO · REPLAY';
  const shell=document.createElement('div');shell.id='hx-shell';
  shell.innerHTML=[
    '<div class="hx-top"><button type="button" class="hx-library-link" aria-label="Открыть библиотеку симуляций">☰ <span>Библиотека</span></button><button type="button" class="hx-select" aria-label="Выбрать симуляцию">',
    '<small>СИМУЛЯЦИЯ</small><span class="hx-title">Tripod</span><b aria-hidden="true">⌄</b>',
    '</button><span class="hx-record">'+sourceLabel+'</span></div>',
    '<div class="hx-controls"><div class="hx-transport">',
    '<button type="button" class="hx-play" aria-label="Пауза">Ⅱ</button>',
    '<div class="hx-progress"><input class="hx-seek" type="range" aria-label="Время записи" min="0" max="60" step=".01" value="0">',
    '<div class="hx-time"><span class="hx-clock">0:00</span><span class="hx-duration">1:00</span></div></div>',
    '<select class="hx-speed" aria-label="Скорость">',
    '<option value=".25">0,25×</option><option value=".5">0,5×</option>',
    '<option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>',
    '</div><button type="button" class="hx-reset">Вернуть камеру</button></div>',
    '<div class="hx-shade" hidden></div>',
    '<section class="hx-sheet" role="dialog" aria-modal="true" aria-label="Библиотека симуляций" hidden>',
    '<div class="hx-grip"></div><div class="hx-sheet-head"><div><strong>Симуляции</strong>',
    '<small>Физические эксперименты</small></div>',
    '<button type="button" class="hx-close" aria-label="Закрыть">×</button></div>',
    '<div class="hx-list" role="list"></div><div class="hx-sheet-foot">',
    '<button type="button" class="hx-refresh">↻ &nbsp; Обновить библиотеку</button>',
    '<div class="hx-sync-message" aria-live="polite"></div></div></section>',
    '<div class="hx-toast" hidden role="status" aria-live="polite"></div>'
  ].join('');
  layout.appendChild(shell);
  const q=(s)=>shell.querySelector(s);
  const title=q('.hx-title'),play=q('.hx-play'),slider=q('.hx-seek');
  const clock=q('.hx-clock'),duration=q('.hx-duration'),speed=q('.hx-speed');
  const shade=q('.hx-shade'),sheet=q('.hx-sheet'),list=q('.hx-list');
  const refresh=q('.hx-refresh'),message=q('.hx-sync-message'),toast=q('.hx-toast');
  const fmt=(value)=>{const x=Math.max(0,Math.floor(Number(value)||0));return Math.floor(x/60)+':'+String(x%60).padStart(2,'0');};
  function show(){
    visible=true;shell.classList.remove('hx-hidden');clearTimeout(hideTimer);
    if(!opened)hideTimer=setTimeout(()=>{visible=false;shell.classList.add('hx-hidden');},4600);
  }
  function notice(t){
    toast.textContent=t;toast.hidden=false;clearTimeout(toastTimer);
    toastTimer=setTimeout(()=>toast.hidden=true,3000);
  }
  function check(items){
    const arr=Array.isArray(items)?items:items?.entries;
    if(!Array.isArray(arr))throw Error('Неверный формат каталога');
    const valid=arr.filter(x=>x&&/^[a-z0-9-]{1,64}$/.test(x.id||'')&&safeHref(x.href)&&
      typeof x.title==='string'&&x.title.length<=100&&typeof x.detail==='string'&&x.detail.length<=180&&
      Number.isFinite(x.duration_s)&&x.duration_s>0);
    if(!valid.length)throw Error('Библиотека пуста');
    return valid;
  }
  const selection=()=>entries.find(x=>x.href===path)||fallback.find(x=>x.href===path)||entries[0];
  function setTitle(){
    const item=selection();if(item)title.textContent=item.title;
    try {if(safeHref(path)&&!libraryView)localStorage.setItem('hexapod:last',path);}catch{}
  }
  function draw(){
    list.replaceChildren();
    for(const item of entries){
      const row=document.createElement('button');row.type='button';row.className='hx-item';
      if(item.site_status==='not-published'){row.disabled=true;row.title='Файлы этого исследования ещё не опубликованы на сайте';row.style.opacity='.55';}
      if(item.href===path)row.classList.add('hx-active');
      const info=document.createElement('span');info.className='hx-item-info';
      const name=document.createElement('strong');name.textContent=item.title;
      const detail=document.createElement('small');detail.textContent=item.detail+(item.site_status==='not-published'?' · ещё не опубликовано':'');
      const meta=document.createElement('span');meta.className='hx-item-meta';
      meta.textContent=item.href===path?'●':fmt(item.duration_s);
      info.append(name,detail);row.append(info,meta);
      row.addEventListener('click',()=>{
        if(item.href===path){close();return;}
        try{localStorage.setItem('hexapod:last',item.href);}catch{}
        location.assign(item.href);
      });list.append(row);
    }
  }
  function open(){opened=true;draw();message.textContent='';shade.hidden=false;sheet.hidden=false;clearTimeout(hideTimer);shell.classList.remove('hx-hidden');q('.hx-close').focus();}
  function close(){opened=false;shade.hidden=true;sheet.hidden=true;q('.hx-select').focus();show();}
  // Open the already-approved bottom-sheet library *in place*.
  // Do not navigate to /hexapod/: older installed shells may redirect to the last replay.
  q('.hx-library-link').addEventListener('click',open);
  q('.hx-select').addEventListener('click',open);
  q('.hx-close').addEventListener('click',close);
  shade.addEventListener('click',close);
  window.addEventListener('keydown',e=>{if(e.key==='Escape'&&opened)close();});
  async function readLibrary(){
    try{
      // All robot families are composed from GitHub main at one pinned commit.
      // Site-local library.json remains only an offline/legacy fallback.
      const {readUnifiedLibrary}=await import('../robot-library/catalog.js');
      const unified=await readUnifiedLibrary();
      entries=check(unified.entries);setTitle();if(opened)draw();return true;
    }catch(error){console.warn('Unified robot library unavailable',error);}
    for(const url of ['/api/library','/hexapod/library.json']){
      try{
        const response=await fetch(url,{cache:'no-store'});
        if(!response.ok)continue;
        entries=check(await response.json());setTitle();if(opened)draw();return true;
      }catch(err){console.warn('Library read',err);}
    }
    entries=fallback;setTitle();return false;
  }
  refresh.addEventListener('click',async()=>{
    refresh.disabled=true;message.textContent='Проверяем обновления…';
    let finished=false, polling=false;
    const started=Date.now(), controller=new AbortController();
    // An unreachable GitHub host must not leave the button disabled forever.
    // The local server may finish its guarded transaction after a client timeout.
    const cutoff=setTimeout(()=>controller.abort(),240000);
    const poll=setInterval(async()=>{
      if(finished||polling)return;
      polling=true;
      const elapsed=Math.floor((Date.now()-started)/1000);
      if(!finished)message.textContent='Проверяем обновления… '+elapsed+' с';
      try{
        const response=await fetch('/api/refresh-status',{cache:'no-store'});
        if(!response.ok)return; // Older installed server: retain elapsed time.
        const state=await response.json();
        if(finished)return;
        const count=state.total?state.checked+' / '+state.total:'';
        const label={
          github:'Проверяем GitHub',
          files:'Проверяем файлы '+count,
          download:'Скачиваем '+String(state.asset||'файл').split('/').pop()+' · '+count,
          install:'Сохраняем проверенную библиотеку',
          done:'Обновление завершено',
          error:'Ошибка синхронизации'
        }[state.phase]||'Проверяем обновления';
        message.textContent=label+' · '+elapsed+' с';
      }catch(err){ /* A temporarily unreachable status endpoint is not fatal. */ }
      finally{polling=false;}
    },1200);
    try{
      const r=await fetch('/api/refresh',{
        method:'POST',headers:{'X-Hexapod-Action':'refresh'},body:'',
        signal:controller.signal
      });
      const result=await r.json().catch(()=>({error:'Обновления доступны только через локальный Termux-сервер'}));
      if(!r.ok)throw Error(result.error||'Ошибка обновления');
      await readLibrary();draw();
      message.textContent=result.added?'Новых симуляций: '+result.added:'Библиотека актуальна';
    }catch(err){
      message.textContent=err.name==='AbortError'
        ?'Нет ответа более 4 минут. Сервер может продолжать обновление: проверьте его состояние в Termux. Старые записи сохранены.'
        :String(err.message||err)+'. Сохранённые записи доступны офлайн.';
    }finally{
      finished=true;clearTimeout(cutoff);clearInterval(poll);refresh.disabled=false;
    }
  });
  play.addEventListener('click',()=>{nativePlay.click();show();});
  slider.addEventListener('input',()=>{
    nativeSeek.value=slider.value;nativeSeek.dispatchEvent(new Event('input',{bubbles:true}));show();
  });
  speed.value=nativeSpeed?.value||'1';
  speed.addEventListener('change',()=>{if(nativeSpeed)nativeSpeed.value=speed.value;show();});
  q('.hx-reset').addEventListener('click',()=>{document.getElementById('reset-camera')?.click();notice('Камера возвращена');show();});
  // Taps toggle playback controls; finger drags stay reserved for OrbitControls.
  let down=null;const activePointers=new Set();
  stage.addEventListener('pointerdown',e=>{
    activePointers.add(e.pointerId);
    down=activePointers.size===1?{id:e.pointerId,x:e.clientX,y:e.clientY,at:performance.now()}:null;
  },{passive:true});
  stage.addEventListener('pointerup',e=>{
    const isSingleTap=activePointers.size===1 && down && e.pointerId===down.id;
    activePointers.delete(e.pointerId);
    if(!isSingleTap)return;
    const diff=Math.hypot(e.clientX-down.x,e.clientY-down.y);
    if(diff<9&&performance.now()-down.at<450){
      if(visible){visible=false;clearTimeout(hideTimer);shell.classList.add('hx-hidden');}else show();
    }
    down=null;
  },{passive:true});
  stage.addEventListener('pointercancel',e=>{activePointers.delete(e.pointerId);down=null;},{passive:true});
  function sync(){
    const time=Number(nativeSeek.value)||0,max=Number(nativeSeek.max)||60;
    if(document.activeElement!==slider){slider.max=String(max);slider.value=String(Math.min(max,time));}
    slider.style.setProperty('--hx-progress',max?100*time/max+'%':'0%');
    clock.textContent=fmt(time);duration.textContent=fmt(max);
    const playing=nativePlay.getAttribute('aria-label')==='Пауза';
    play.textContent=playing?'Ⅱ':'▶';play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');
    requestAnimationFrame(sync);
  }
  // Video view: no debug contact rings or path unless explicitly enabled later.
  for(const id of ['contacts','path']){
    const checkbox=document.getElementById(id);
    if(checkbox?.checked){checkbox.checked=false;checkbox.dispatchEvent(new Event('change',{bubbles:true}));}
  }
  setTitle();readLibrary();requestAnimationFrame(sync);show();
  if(libraryView)open();
})();
