/* One-time browser bridge for the existing private Sites player. */
(() => {
  'use strict';
  let promise = null;
  window.alfredGithubCatalogBoot = () => {
    if (promise) return promise;
    promise = (async () => {
      if (!('serviceWorker' in navigator) || !window.isSecureContext) return false;
      await navigator.serviceWorker.register('/github-live-sw.js', {scope:'/'});
      await navigator.serviceWorker.ready;
      if (navigator.serviceWorker.controller?.scriptURL?.endsWith('/github-live-sw.js')) return true;
      // First install: skipWaiting + clients.claim makes this tab live.
      return await new Promise(resolve => {
        let finished = false;
        const done = ok => {
          if (finished) return;
          finished = true;
          clearTimeout(timer);
          navigator.serviceWorker.removeEventListener('controllerchange', changed);
          resolve(ok);
        };
        const changed = () => done(!!navigator.serviceWorker.controller?.scriptURL?.endsWith('/github-live-sw.js'));
        const timer = setTimeout(() => done(false), 6000);
        navigator.serviceWorker.addEventListener('controllerchange', changed);
        changed();
      });
    })().catch(error => {
      console.warn('Alfred GitHub live catalog unavailable; bundled recordings remain available.', error);
      return false;
    });
    return promise;
  };
})();
