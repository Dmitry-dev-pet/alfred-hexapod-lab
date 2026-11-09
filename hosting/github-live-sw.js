/* Alfred Robot Player: manifest-verified, read-only GitHub replay library.
 * Scope: only /hexapod/library.json and allowlisted manifest assets.
 * Never writes back to GitHub, evaluates a backend API, or rewrites existing IDs.
 */
'use strict';
const GH_OWNER = 'Dmitry-dev-pet';
const GH_REPO = 'alfred-hexapod-lab';
const GH_BRANCH = 'feature/android-local-offline-viewer-20261104';
const GH_API = 'https://api.github.com/repos/'+GH_OWNER+'/'+GH_REPO+'/branches/'+GH_BRANCH;
const GH_RAW = 'https://raw.githubusercontent.com/'+GH_OWNER+'/'+GH_REPO+'/';
const CACHE_NAME = 'alfred-robot-github-catalog-v1';
const META_KEY = '/__alfred_github_catalog_v1.json';
const MAX_CATALOG_BYTES = 250_000;
const MAX_FILE_BYTES = 8_000_000;
const SHA = /^[a-f0-9]{40}$/;
const DIGEST = /^[a-f0-9]{64}$/;
const ID = /^[a-z0-9-]{1,64}$/;
const PATH = /^hexapod\/(?:[A-Za-z0-9_-]+\/)*[A-Za-z0-9_.-]+\.(?:html|js|css|json)$/;
const MIME = { html:'text/html; charset=utf-8', js:'text/javascript; charset=utf-8', css:'text/css; charset=utf-8', json:'application/json; charset=utf-8' };
const SAME_ORIGIN = self.location.origin;
let current = null;
let currentCheckedAt = 0;
let lastCatalogWasOffline = false;
let inflight = null;

function isRecord(x) { return x !== null && typeof x === 'object' && !Array.isArray(x); }
function validateCatalog(doc) {
  if (!isRecord(doc) || doc.schema !== 'hexapod-library/v1' ||
      !Number.isInteger(doc.revision) || doc.revision < 1 || doc.revision > 100000 ||
      !Array.isArray(doc.entries) || doc.entries.length < 1 || doc.entries.length > 120) {
    throw Error('Invalid GitHub catalogue');
  }
  const ids = new Set(), routes = new Set(), files = new Map();
  for (const entry of doc.entries) {
    if (!isRecord(entry) || !ID.test(entry.id) || ids.has(entry.id) ||
        typeof entry.title !== 'string' || entry.title.length > 100 ||
        typeof entry.detail !== 'string' || entry.detail.length > 180 ||
        typeof entry.href !== 'string' ||
        !/^\/hexapod\/(?:[a-z0-9-]+\/)?$/.test(entry.href) ||
        routes.has(entry.href) ||
        !Number.isFinite(entry.duration_s) || entry.duration_s <= 0 ||
        !Array.isArray(entry.files) || entry.files.length < 1 || entry.files.length > 30) {
      throw Error('Invalid GitHub catalogue entry');
    }
    ids.add(entry.id); routes.add(entry.href);
    for (const file of entry.files) {
      if (!isRecord(file) || typeof file.path !== 'string' || !PATH.test(file.path) ||
          file.path.split('/').some(p => !p || p === '.' || p === '..' || p.startsWith('.')) ||
          !DIGEST.test(file.sha256) || !Number.isInteger(file.size) ||
          file.size < 1 || file.size > MAX_FILE_BYTES) {
        throw Error('Unsafe GitHub catalogue asset');
      }
      const prev = files.get(file.path);
      if (prev && (prev.sha256 !== file.sha256 || prev.size !== file.size)) {
        throw Error('Conflicting immutable asset');
      }
      files.set(file.path, {sha256:file.sha256, size:file.size});
    }
    // Legacy entries may share a bundled page not listed in entry.files.
    // The worker only proxies assets explicitly listed and SHA-verified.
  }
  if (files.size > 600) throw Error('Too many catalogue assets');
  return {document:doc, files};
}
async function boundedFetch(url, maxSize) {
  const response = await fetch(url, {cache:'no-store', mode:'cors', redirect:'error'});
  if (!response.ok || response.type === 'opaque') throw Error('GitHub resource unavailable: '+response.status);
  const declared = Number(response.headers.get('content-length'));
  if (declared && declared > maxSize) throw Error('Oversized GitHub resource');
  const bytes = await response.arrayBuffer();
  if (bytes.byteLength < 1 || bytes.byteLength > maxSize) throw Error('Invalid GitHub resource length');
  return bytes;
}
function metaURL() { return new URL(META_KEY, SAME_ORIGIN); }
async function cachedCatalog() {
  if (current) return current;
  const response = await (await caches.open(CACHE_NAME)).match(metaURL());
  if (!response) return null;
  try {
    const item = await response.json();
    if (!SHA.test(item.sha)) return null;
    const verified = validateCatalog(item.catalog);
    current = {...verified, sha:item.sha};
    return current;
  } catch (_) { return null; }
}
async function refreshCatalog(force) {
  if (inflight) return inflight;
  if (!force && current && Date.now() - currentCheckedAt < 30_000) return current;
  inflight = (async () => {
    const existing = await cachedCatalog();
    try {
      const headBytes = await boundedFetch(GH_API, 100_000);
      const head = JSON.parse(new TextDecoder().decode(headBytes));
      const sha = head && head.commit && head.commit.sha;
      if (typeof sha !== 'string' || !SHA.test(sha)) throw Error('Invalid GitHub branch commit');
      if (existing && existing.sha === sha) {
        lastCatalogWasOffline = false;
        currentCheckedAt = Date.now();
        return existing;
      }
      // Fetch only at immutable commit SHAs. No branch/ref race between reads.
      const bytes = await boundedFetch(GH_RAW + sha + '/docs/hexapod/library.json', MAX_CATALOG_BYTES);
      const verified = validateCatalog(JSON.parse(new TextDecoder().decode(bytes)));
      if (existing && verified.document.revision < existing.document.revision) {
        throw Error('Refusing GitHub catalogue revision rollback');
      }
      const snapshot = {...verified, sha};
      await (await caches.open(CACHE_NAME)).put(metaURL(), new Response(JSON.stringify({sha, catalog:verified.document}), {
        headers:{'Content-Type':'application/json'}
      }));
      current = snapshot;
      lastCatalogWasOffline = false;
      currentCheckedAt = Date.now();
      return snapshot;
    } catch (error) {
      if (existing) {
        lastCatalogWasOffline = true;
        currentCheckedAt = Date.now(); // Bound repeat requests while offline.
        return existing;
      }
      throw error;
    }
  })();
  try { return await inflight; }
  finally { inflight = null; }
}
function mime(path) { return MIME[path.slice(path.lastIndexOf('.')+1)] || 'application/octet-stream'; }
async function sha256(buf) {
  const hash = await crypto.subtle.digest('SHA-256', buf);
  return Array.from(new Uint8Array(hash), b => b.toString(16).padStart(2,'0')).join('');
}
function publicResponse(bytes, path) {
  return new Response(bytes, {status:200, headers:{
    'Content-Type':mime(path),
    'Cache-Control':'no-store',
    'X-Content-Type-Options':'nosniff',
    'X-Alfred-Catalog-Source':'github'
  }});
}
async function serveAsset(item, path, file) {
  const key = new URL('/__alfred_verified_gh/'+item.sha+'/'+file.sha256+'/'+path, SAME_ORIGIN);
  const cache = await caches.open(CACHE_NAME);
  const hit = await cache.match(key);
  if (hit) return hit;
  try {
    const buf = await boundedFetch(GH_RAW+item.sha+'/docs/'+path, file.size);
    if (buf.byteLength !== file.size || await sha256(buf) !== file.sha256) {
      throw Error('GitHub asset failed SHA-256 integrity check');
    }
    let payload = buf;
    if (path.endsWith('.html')) {
      // The original static Sites build uses integrity-verified local Three.js.
      let html = new TextDecoder().decode(buf);
      html = html.replaceAll('https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js',
                             '/vendor/three/build/three.module.js')
                 .replaceAll('https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/',
                             '/vendor/three/examples/jsm/');
      payload = new TextEncoder().encode(html);
    }
    const response = publicResponse(payload, path);
    await cache.put(key, response.clone());
    return response;
  } catch (error) {
    // Never serve an unverified remote file. An existing static bundled
    // route can still be served by the hosting platform, but callers are
    // told it is not a GitHub-verified response.
    return new Response('Verified replay resource unavailable', {status:503, headers:{'Content-Type':'text/plain; charset=utf-8'}});
  }
}
async function handle(event) {
  const url = new URL(event.request.url);
  const pathname = url.pathname;
  if (pathname === '/hexapod/library.json') {
    try {
      const item = await refreshCatalog(true);
      const src = lastCatalogWasOffline ? 'cached-github' : 'github';
      return new Response(JSON.stringify(item.document), {status:200, headers:{
        'Content-Type':'application/json; charset=utf-8',
        'Cache-Control':'no-store',
        'X-Alfred-Catalog-Source':src,
        'X-Alfred-Catalog-Commit':item.sha
      }});
    } catch (error) {
      return fetch(event.request); // Bundled site snapshot remains usable.
    }
  }
  // Never intercept site home, CSS/shell/vendor, other applications, or API.
  if (!pathname.startsWith('/hexapod/') || pathname === '/hexapod/') return fetch(event.request);
  let item;
  try { item = await refreshCatalog(false); }
  catch (error) { return fetch(event.request); }
  let path = pathname.slice(1);
  if (pathname.endsWith('/')) path += 'index.html';
  const file = item.files.get(path);
  if (!file) return fetch(event.request); // Local shared assets and legacy routes.
  return serveAsset(item, path, file);
}
self.addEventListener('install', event => event.waitUntil(self.skipWaiting()));
self.addEventListener('activate', event => event.waitUntil(self.clients.claim()));
self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET' || new URL(event.request.url).origin !== SAME_ORIGIN) return;
  const pathname = new URL(event.request.url).pathname;
  if (!pathname.startsWith('/hexapod/')) return;
  event.respondWith(handle(event));
});