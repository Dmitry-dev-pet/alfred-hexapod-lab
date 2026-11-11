import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile, access} from 'node:fs/promises';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';

// This is a controller/DOM-adapter test. It does not emulate layout, WebGL or touch.
const root = new URL('../', import.meta.url);
let source = new URL('docs/', root);
try { await access(new URL('replay/player.js', source)); }
catch { source = new URL('dist/', root); }
const html = await readFile(new URL('replay/index.html', source), 'utf8');
const legacy = JSON.parse(await readFile(new URL('robots/catalog.json', source), 'utf8'));
const recordings = JSON.parse(await readFile(new URL('replay/catalog.json', source), 'utf8'));
const fixture = await readFile(new URL('replay/fixtures/freenove-stand-torso-2s.robotreplay', source));
const sha = 'a'.repeat(40);

class Element {
  constructor(tag, document) {
    Object.assign(this, {tagName: tag.toUpperCase(), document, children: [], listeners: {}, attributes: {},
      value: '', hidden: false, disabled: false, open: false, dataset: {}, scrollTop: 0});
  }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = [...children]; this.text = ''; }
  set textContent(value) { this.text = String(value); this.children = []; }
  get textContent() { return (this.text || '') + this.children.map(child => child.textContent).join(''); }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  addEventListener(name, fn) { (this.listeners[name] ||= []).push(fn); }
  focus() { this.document.activeElement = this; }
  click() { return this.emit('click'); }
  async emit(name, props = {}) {
    const event = {button: 0, target: this, preventDefault() { this.defaultPrevented = true; }, ...props};
    for (const fn of [...(this.listeners[name] || []), ...(this['on' + name] ? [this['on' + name]] : [])])
      await fn(event);
    return event;
  }
}
function makeDocument() {
  const document = {
    elements: new Map(),
    createElement(tag) { return new Element(tag, document); },
    getElementById(id) {
      assert.ok(document.elements.has(id), 'Control is present in the real HTML: ' + id);
      return document.elements.get(id);
    }
  };
  document.documentElement = document.createElement('html');
  document.documentElement.dataset.view = 'library';
  for (const match of html.matchAll(/<([a-z][a-z0-9-]*)\b([^>]*\bid="([^"]+)"[^>]*)>/gi)) {
    const element = document.createElement(match[1]);
    element.hidden = /\bhidden\b/.test(match[2]);
    element.disabled = /\bdisabled\b/.test(match[2]);
    document.elements.set(match[3], element);
  }
  document.getElementById('speed').value = '1';
  return document;
}
class Vector {
  constructor(x = 0, y = 0, z = 0) { this.set(x, y, z); }
  set(x, y, z) { Object.assign(this, {x, y, z}); return this; }
  copy(v) { return this.set(v.x, v.y, v.z); }
  clone() { return new Vector(this.x, this.y, this.z); }
  add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
}
class Quaternion { set() { return this; } copy() { return this; } slerp() { return this; } }
class Object3D {
  constructor() { this.children = []; this.position = new Vector(); this.rotation = {}; this.quaternion = new Quaternion(); }
  add(...items) { this.children.push(...items); }
  remove(item) { this.children = this.children.filter(value => value !== item); }
  clear() { this.children = []; }
  traverse(fn) { fn(this); for (const child of this.children) child.traverse?.(fn); }
  localToWorld(v) { return v; }
}
class Mesh extends Object3D { constructor(geometry, material) { super(); Object.assign(this, {geometry, material}); } }
class Disposable { dispose() {} }
const three = {
  Scene: Object3D, Group: Object3D, PerspectiveCamera: Object3D, Vector3: Vector, Quaternion,
  WebGLRenderer: class { constructor() { throw Error('No WebGL in controller tests'); } },
  Color: class {}, PlaneGeometry: Disposable, BoxGeometry: Disposable,
  MeshStandardMaterial: Disposable, Mesh,
  HemisphereLight: Object3D, DirectionalLight: Object3D
};
async function until(predicate) {
  for (let i = 0; i < 2000; i++) {
    if (predicate()) return;
    await new Promise(resolve => setTimeout(resolve, 1));
  }
  assert.fail('Controller did not reach the expected state');
}
function descendants(element) { return element.children.flatMap(child => [child, ...descendants(child)]); }

async function environment({defer = false, search = ''} = {}) {
  assert.ok(vm.SourceTextModule, 'Run with node --experimental-vm-modules --test tests/robot-player-ui.test.mjs');
  const document = makeDocument(), handlers = {}, requests = [];
  let tick, resolveDownload;
  const delayed = new Promise(resolve => { resolveDownload = resolve; });
  const location = {href: 'https://robot.test/replay/' + search, search};
  const history = {pushState(_state, _title, value) {
    const url = new URL(value, location.href);
    Object.assign(location, {href: url.href, search: url.search});
  }};
  const fetch = async input => {
    const url = String(input); requests.push(url);
    if (url.endsWith('/branches/main')) return Response.json({commit: {sha}});
    if (url.endsWith('/docs/robots/catalog.json')) return Response.json(legacy);
    if (url.endsWith('/docs/replay/catalog.json')) return Response.json(recordings);
    if (url.endsWith('freenove-stand-torso-2s.robotreplay'))
      return defer ? delayed : new Response(fixture);
    if (url === 'https://files.test/bad.robotreplay') return new Response('broken');
    throw Error('Unexpected fetch: ' + url);
  };
  const context = vm.createContext({
    document, location, history, fetch, console, URL, URLSearchParams, Response, AbortSignal,
    TextDecoder, TextEncoder, Blob, DecompressionStream, crypto: webcrypto, atob,
    Uint8Array, Uint32Array, Float32Array, Float64Array, ArrayBuffer, devicePixelRatio: 1,
    ResizeObserver: class { observe() {} },
    requestAnimationFrame: fn => { tick = fn; },
    window: {addEventListener(name, fn) { (handlers[name] ||= []).push(fn); }}
  });
  const modules = new Map();
  async function moduleAt(url) {
    if (modules.has(url.href)) return modules.get(url.href);
    const pending = (async () => {
    const module = new vm.SourceTextModule(await readFile(url, 'utf8'), {context, identifier: url.href});
    await module.link(async (specifier, parent) => {
      if (specifier === 'three' || specifier.includes('OrbitControls')) {
        const exports = specifier === 'three' ? three : {OrbitControls: class {}};
        return new vm.SyntheticModule(Object.keys(exports), function() {
          for (const [key, value] of Object.entries(exports)) this.setExport(key, value);
        }, {context});
      }
      return moduleAt(new URL(specifier, parent.identifier));
    });
    return module;
    })();
    modules.set(url.href, pending);
    return pending;
  }
  await (await moduleAt(new URL('replay/player.js', source))).evaluate();
  await until(() => document.documentElement.dataset.librarySource);
  // Give initial deep-link navigation a chance to start.
  await new Promise(resolve => setTimeout(resolve, 0));
  const $ = id => document.getElementById(id);
  return {$, document, context, location, requests,
    cards: () => descendants($('recordings')).filter(e => e.className?.split(' ').includes('recording-link')),
    finishDownload: () => resolveDownload(new Response(fixture)),
    frame: now => tick(now),
    pop: async query => {
      location.href = 'https://robot.test/replay/' + query; location.search = query;
      for (const fn of handlers.popstate || []) await fn();
    }
  };
}

test('real catalog is searchable, family-filtered, and pending entries are not links', async () => {
  const env = await environment(), {$, document} = env;
  assert.equal(env.cards().length, 82);
  assert.equal(env.cards().filter(e => e.attributes['aria-disabled'] === 'true').length, 4);
  assert.ok(env.cards().filter(e => e.attributes['aria-disabled']).every(e => !e.href));
  $('search').value = '  HEXARTH   ступень  ';
  await $('search').emit('input');
  assert.equal(env.cards().length, 1);
  assert.match(env.cards()[0].textContent, /подбор подъёма/);
  $('search').value = 'подъем';
  await $('search').emit('input');
  assert.ok(env.cards().length > 0);
  assert.ok(env.cards().some(card => card.textContent.includes('подъём')));
  $('search').value = 'подьем'; // ё and е are equivalent, not a fuzzy spelling correction.
  await $('search').emit('input');
  assert.equal(env.cards().length, 0);
  assert.equal($('no-results').hidden, false);
  await $('clear-search').click();
  assert.equal(env.cards().length, 82);
  assert.equal(document.activeElement, $('search'));
  $('robot-filter').value = 'MicroDuck';
  await $('robot-filter').emit('change');
  const count = env.cards().length;
  assert.ok(count > 0 && count < 82);
  await $('refresh-library').click();
  assert.equal($('robot-filter').value, 'MicroDuck');
  assert.equal(env.cards().length, count);
  assert.equal($('refresh-library').disabled, false);
});

test('same-page recording navigation, seek, pause, return and history use the real decoder', async () => {
  const env = await environment(), {$, document} = env;
  $('robot-filter').value = 'Freenove FNK0052'; await $('robot-filter').emit('change');
  const card = env.cards().find(e => e.href?.includes('freenove-stand-torso-2s'));
  const modified = await card.emit('click', {ctrlKey: true});
  assert.equal(modified.defaultPrevented, undefined);
  await card.click();
  await until(() => document.documentElement.dataset.replayReady === 'true');
  assert.equal(document.documentElement.dataset.view, 'player');
  assert.match(env.location.search, /recording=freenove-stand-torso-2s/);
  assert.equal($('play').disabled, false);
  assert.match($('title').textContent, /Freenove/);
  $('seek').value = '1'; await $('seek').emit('input');
  assert.equal(document.documentElement.dataset.time, '1');
  await $('play').click();
  assert.equal($('play').attributes['aria-label'], 'Воспроизвести');
  env.frame(0); env.frame(100);
  assert.equal(document.documentElement.dataset.time, '1');
  await $('play').click(); env.frame(200); env.frame(300);
  assert.ok(Number(document.documentElement.dataset.time) > 1);
  $('empty').scrollTop = 120;
  await $('library').click();
  assert.equal(document.documentElement.dataset.view, 'library');
  assert.equal($('robot-filter').value, 'Freenove FNK0052');
  assert.equal($('empty').scrollTop, 120);
  assert.equal(env.location.search, '');
  await env.pop('?recording=freenove-stand-torso-2s');
  assert.equal(document.documentElement.dataset.view, 'player');
  assert.equal($('play').disabled, false);
});

test('returning to the library cancels an in-flight recording install', async () => {
  const env = await environment({defer: true}), {$, document} = env;
  const card = env.cards().find(e => e.href?.includes('freenove-stand-torso-2s'));
  await card.click();
  assert.equal(document.documentElement.dataset.view, 'player');
  await $('library').click();
  env.finishDownload();
  await new Promise(resolve => setTimeout(resolve, 20));
  assert.equal(document.documentElement.dataset.view, 'library');
  assert.notEqual(document.documentElement.dataset.replayReady, 'true');
});

test('a failed URL or local file preserves the loaded recording and enables retry', async () => {
  const env = await environment({search: '?recording=freenove-stand-torso-2s'}), {$, document} = env;
  await until(() => document.documentElement.dataset.replayReady === 'true');
  $('url').value = 'https://files.test/bad.robotreplay';
  await $('url-form').emit('submit');
  await until(() => !$('error').hidden);
  assert.equal($('error').hidden, false);
  assert.equal($('play').disabled, false);
  assert.equal(document.documentElement.dataset.replayReady, 'true');
  $('file').files = [{name: 'bad.robotreplay', size: 1, async arrayBuffer() { throw Error('disk read failed'); }}];
  await $('file').emit('change');
  await until(() => $('error').textContent.includes('disk read failed'));
  assert.equal($('play').disabled, false);
  await $('library').click();
  assert.equal($('error').hidden, true);
  assert.equal(document.documentElement.dataset.view, 'library');
});
