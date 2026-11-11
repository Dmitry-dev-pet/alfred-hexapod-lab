/* Keep the library and recorded videos usable when browser 3D is unavailable. */
window.addEventListener('error', event => {
  if (!/WebGL context/i.test(String(event.message || event.error?.message || ''))) return;
  if (document.querySelector('#webgl-unavailable')) return;
  const stage = document.querySelector('.stage') || document.querySelector('#stage');
  if (!stage) return;
  const panel = document.createElement('section');
  panel.id = 'webgl-unavailable'; panel.setAttribute('role','status');
  panel.style.cssText='position:absolute;inset:25% 6% auto;z-index:20;max-width:520px;margin:auto;padding:24px;border:1px solid #415463;border-radius:16px;background:#18232d;color:#e9f0f5;font:16px/1.6 system-ui';
  const title = document.createElement('strong'); title.textContent='3D-просмотр недоступен в этом браузере';
  const text = document.createElement('p'); text.textContent='Попробуйте другой браузер или откройте готовые видеозаписи.';
  const link = document.createElement('a'); link.href='/videos/'; link.textContent='Открыть видеозаписи →'; link.style.color='#a8dfff';
  panel.append(title,text,link); stage.append(panel);
  const disable = () => {
    for (const control of document.querySelectorAll('#play,#seek,#speed,#reset-camera,.hx-play,.hx-seek,.hx-speed,.hx-reset')) control.disabled=true;
  };
  disable(); setTimeout(disable,0);
});
