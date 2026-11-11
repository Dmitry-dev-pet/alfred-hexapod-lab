// Immutable GitHub transport for data-only recordings. No simulation or model code.
const REPOSITORY = 'Dmitry-dev-pet/alfred-hexapod-lab';
const RAW = 'https://raw.githubusercontent.com/' + REPOSITORY + '/';
const CACHE = 'alfred-github-recording-bytes-v1';
const MAX = 100 * 1024 * 1024;
const CANDIDATE = /^\/(?:assets\/robot-(?:replays|motions)\/|hexapod\/data\/|replay\/(?:records|fixtures)\/)/;
const PATH = /^\/(?:assets\/robot-(?:replays|motions)\/[a-z0-9_./-]+\.(?:bin|json|gz)|hexapod\/data\/[a-z0-9_.-]+\.json|replay\/(?:records|fixtures)\/[a-z0-9_.-]+\.robotreplay)$/i;
const HASH = /^[a-f0-9]{64}$/;

export function validateRecordingStore(doc) {
  if (doc?.schema !== 'robot-recording-store/v1' || doc.repository !== REPOSITORY ||
      !/^[a-f0-9]{40}$/.test(doc.commit) || doc.prefix !== 'recordings/site-v33' ||
      !Array.isArray(doc.files) || !doc.files.length || doc.files.length > 1000)
    throw Error('Неверный источник записей GitHub');
  const paths = new Set();
  for (const file of doc.files) {
    if (!PATH.test(file.path) || file.path.split('/').slice(1).some(p => !p || p === '.' || p === '..' || p.startsWith('.')) ||
        paths.has(file.path) || !Number.isSafeInteger(file.size) || file.size < 1 || file.size > MAX || !HASH.test(file.sha256))
      throw Error('Неверный файл записи GitHub');
    paths.add(file.path);
  }
  return doc;
}
const digest = async bytes => [...new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))]
  .map(value => value.toString(16).padStart(2, '0')).join('');
async function boundedBytes(response, size) {
  if (!response.ok || response.type === 'opaque') throw Error('Запись недоступна: HTTP ' + response.status);
  if (Number(response.headers.get('content-length')) > size) throw Error('Превышен размер записи');
  if (!response.body) {
    const bytes = await response.arrayBuffer();
    if (bytes.byteLength > size) throw Error('Превышен размер записи');
    return bytes;
  }
  const reader = response.body.getReader(), chunks = [];
  let count = 0;
  try {
    for (;;) {
      const {value, done} = await reader.read();
      if (done) break;
      count += value.byteLength;
      if (count > size) throw Error('Превышен размер записи');
      chunks.push(value);
    }
  } catch (error) {
    // A cloned response can have an unread tee branch; cancellation must not delay rejection.
    reader.cancel().catch(() => {});
    throw error;
  }
  const out = new Uint8Array(count);
  let offset = 0;
  for (const chunk of chunks) { out.set(chunk, offset); offset += chunk.byteLength; }
  return out.buffer;
}
async function checked(response, file) {
  const bytes = await boundedBytes(response, file.size);
  if (bytes.byteLength !== file.size || await digest(bytes) !== file.sha256)
    throw Error('Контрольная сумма записи GitHub не совпала');
  return bytes;
}
const mime = path => path.endsWith('.json') ? 'application/json' :
  path.endsWith('.gz') ? 'application/gzip' : 'application/octet-stream';

export function createRecordingFetch({
  fetchImpl = globalThis.fetch.bind(globalThis),
  origin = globalThis.location?.origin,
  cacheStorage = globalThis.caches,
  manifest: suppliedManifest,
  manifestURL = '/robots/recording-assets.json'
} = {}) {
  const base = new URL(origin);
  let manifestPromise;
  async function readManifest() {
    if (!manifestPromise) manifestPromise = (async () => {
      if (suppliedManifest) return validateRecordingStore(suppliedManifest);
      const response = await fetchImpl(new URL(manifestURL, base), {
        cache: 'no-store', credentials: 'same-origin', redirect: 'error',
        signal: AbortSignal.timeout(15000)
      });
      const bytes = await boundedBytes(response, 300000);
      return validateRecordingStore(JSON.parse(new TextDecoder().decode(bytes)));
    })().catch(error => { manifestPromise = null; throw error; });
    return manifestPromise;
  }
  return async function recordingFetch(input, init) {
    const url = new URL(typeof input?.url === 'string' ? input.url : String(input), base);
    const method = (init?.method || input?.method || 'GET').toUpperCase();
    if (url.origin !== base.origin || !CANDIDATE.test(url.pathname) || !['GET', 'HEAD'].includes(method))
      return fetchImpl(input, init);
    const manifest = await readManifest();
    const file = manifest.files.find(file => file.path === url.pathname);
    if (!file) return fetchImpl(input, init);
    const signal = init?.signal || input?.signal;
    signal?.throwIfAborted?.();
    const remote = RAW + manifest.commit + '/' + manifest.prefix + file.path;
    const headers = {
      'Content-Type': mime(file.path), 'Content-Length': String(file.size),
      'Cache-Control': 'public, max-age=31536000, immutable',
      'X-Content-Type-Options': 'nosniff',
      'X-Alfred-Recording-Source': 'github',
      'X-Alfred-Recording-Commit': manifest.commit,
      'X-Content-SHA256': file.sha256
    };
    const cache = await cacheStorage?.open(CACHE).catch(() => null);
    let bytes;
    if (cache) {
      const hit = await cache.match(remote);
      if (hit) {
        try { bytes = await checked(hit, file); }
        catch { await cache.delete(remote).catch(() => {}); }
      }
    }
    if (!bytes) {
      const response = await fetchImpl(remote, {
        mode: 'cors', credentials: 'omit', redirect: 'error', cache: 'force-cache',
        signal: signal ? AbortSignal.any([signal, AbortSignal.timeout(90000)]) : AbortSignal.timeout(90000)
      });
      bytes = await checked(response, file);
      if (cache) await cache.put(remote, new Response(bytes, {headers})).catch(() => {});
    }
    signal?.throwIfAborted?.();
    return new Response(method === 'HEAD' ? null : bytes, {status: 200, headers});
  };
}
