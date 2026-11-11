import test from 'node:test';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';
let source = new URL('../docs/', import.meta.url);
try { await readFile(new URL('recording-store.js', source)); }
catch { source = new URL('../dist/', import.meta.url); }
const {validateRecordingStore, createRecordingFetch} = await import(new URL('recording-store.js', source));
const sha = 'a'.repeat(40), bytes = new Uint8Array([3, 1, 4, 1, 5]);
const file = {path: '/assets/robot-replays/microduck-falls-v1/test.bin', size: bytes.length,
  sha256: createHash('sha256').update(bytes).digest('hex')};
const manifest = {schema: 'robot-recording-store/v1', repository: 'Dmitry-dev-pet/alfred-hexapod-lab',
  commit: sha, prefix: 'recordings/site-v33', files: [file]};
const origin = 'https://robot.test';
const remote = 'https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/' + sha + '/recordings/site-v33' + file.path;
function cacheStorage() {
  const entries = new Map();
  return {entries, async open() { return {
    async match(k) { return entries.get(String(k))?.clone(); },
    async put(k, r) { entries.set(String(k), r.clone()); },
    async delete(k) { entries.delete(String(k)); }
  }; }};
}

test('immutable GitHub bytes preserve the legacy address and omit Site credentials', async () => {
  const requests = [];
  const fetch = createRecordingFetch({origin, manifest, fetchImpl: async (url, options) => {
    requests.push({url: String(url), options}); return new Response(bytes);
  }});
  const response = await fetch(origin + file.path, {credentials: 'include'});
  assert.deepEqual(new Uint8Array(await response.arrayBuffer()), bytes);
  assert.equal(requests[0].url, remote);
  assert.equal(requests[0].options.credentials, 'omit');
  assert.equal(requests[0].options.redirect, 'error');
  assert.equal(response.headers.get('X-Alfred-Recording-Source'), 'github');
  assert.equal(response.headers.get('X-Content-SHA256'), file.sha256);
});

test('cache survives offline, is reverified and rejects poison or corrupt downloads', async () => {
  const cache = cacheStorage();
  let online = true, requests = 0;
  const fetch = createRecordingFetch({origin, manifest, cacheStorage: cache, fetchImpl: async () => {
    requests++; if (!online) throw Error('offline'); return new Response(bytes);
  }});
  await (await fetch(origin + file.path)).arrayBuffer();
  online = false;
  await (await fetch(origin + file.path)).arrayBuffer();
  assert.equal(requests, 1);
  cache.entries.set(remote, new Response('poison'));
  await assert.rejects(fetch(origin + file.path), /offline/);
  const corrupt = createRecordingFetch({origin, manifest, fetchImpl: async () => new Response(new Uint8Array(bytes.length))});
  await assert.rejects(corrupt(origin + file.path), /сумма/);
  const oversized = createRecordingFetch({origin, manifest, fetchImpl: async () => new Response(new Uint8Array(bytes.length + 1))});
  await assert.rejects(oversized(origin + file.path), /размер/);
});

test('unsafe paths, duplicate aliases and arbitrary storage hosts are rejected', () => {
  validateRecordingStore(manifest);
  for (const change of [
    d => d.files[0].path = '/assets/robot-replays/../private.json',
    d => d.files[0].path = '/api/session.json',
    d => d.files.push(d.files[0]),
    d => d.repository = 'other/owner',
    d => d.prefix = 'https://files.test',
    d => d.commit = 'main'
  ]) {
    const bad = structuredClone(manifest); change(bad);
    assert.throws(() => validateRecordingStore(bad));
  }
});

test('shell, models, other origins, unlisted paths and writes stay on their original transport', async () => {
  const calls = [];
  const fetch = createRecordingFetch({origin, manifest, fetchImpl: async (input, init) => {
    calls.push({input, init}); return new Response('native');
  }});
  for (const url of ['/replay/player.js', '/assets/robot-models/geometry.json', 'https://elsewhere.test' + file.path,
    '/assets/robot-replays/not-listed.bin'])
    assert.equal(await (await fetch(new URL(url, origin))).text(), 'native');
  const request = new Request(origin + file.path, {method: 'POST', body: 'test'});
  assert.equal(await (await fetch(request)).text(), 'native');
  assert.equal(calls.at(-1).input, request);
});

test('the first page visit loads a recording before service-worker control exists', async () => {
  const boot = await readFile(new URL('github-live-boot.js', source), 'utf8');
  const relativeFile = {...file, path: '/hexapod/data/test.json'};
  const bootManifest = {...manifest, files: [file, relativeFile]};
  const calls = [], window = {
    fetch: async input => {
      const url = String(input); calls.push(url);
      return url.endsWith('/robots/recording-assets.json') ? Response.json(bootManifest) : new Response(bytes);
    }
  };
  vm.runInNewContext(boot, {window, navigator: {}, location: {origin},
    document: {baseURI: origin + '/hexapod/microduck-falls-v1/'}, URL, console},
  {importModuleDynamically: () => import(new URL('recording-store.js', source))});
  const response = await window.fetch(origin + file.path);
  assert.deepEqual(new Uint8Array(await response.arrayBuffer()), bytes);
  assert.deepEqual(calls, [origin + '/robots/recording-assets.json', remote]);
  const relative = await window.fetch('../data/test.json');
  assert.deepEqual(new Uint8Array(await relative.arrayBuffer()), bytes);
  assert.equal(calls.at(-1), 'https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/' + sha + '/recordings/site-v33/hexapod/data/test.json');
});

test('every migrated file has one integrity-checked alias', async () => {
  const doc = validateRecordingStore(JSON.parse(await readFile(new URL('robots/recording-assets.json', source), 'utf8')));
  assert.equal(doc.files.length, 377);
  assert.equal(doc.files.reduce((n, f) => n + f.size, 0), 244287539);
  assert.equal(doc.files.filter(f => f.path.endsWith('.robotreplay')).length, 3);
});
