// Install before replay modules run, including the first visit before a worker claims the tab.
(() => {
  if (window.alfredRecordingFetchInstalled) return;
  const originalFetch = window.fetch.bind(window);
  const ready = import('/recording-store.js').then(module => module.createRecordingFetch({
    fetchImpl: originalFetch, origin: location.origin
  }));
  // Keep ordinary shell, vendor, model, API and other-site requests on the native path.
  window.fetch = (input, init) => {
    const url = new URL(typeof input?.url === 'string' ? input.url : String(input), document.baseURI);
    return url.origin === location.origin &&
      /^\/(?:assets\/robot-(?:replays|motions)\/|hexapod\/data\/|replay\/(?:records|fixtures)\/)/.test(url.pathname)
      ? ready.then(fetchRecording => fetchRecording(typeof input?.url === 'string' ? input : url.href, init))
      : originalFetch(input, init);
  };
  window.alfredRecordingFetchInstalled = true;
})();
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
    }).then(ok => {
      // A slow first claim can finish after the six-second wait. Allow retry.
      if (!ok) promise = null;
      return ok;
    });
    return promise;
  };
  // Start registration before a replay engine or its shared shell executes.
  window.alfredGithubCatalogBoot();
})();
