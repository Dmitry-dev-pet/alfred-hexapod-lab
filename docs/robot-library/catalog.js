// Canonical library view: compose all legacy pages and generic robot-replay/v1 records.
// GitHub main is the authority for the two format-specific manifests and the
// small inventory of still-Site-only historical recordings. No extra database.
const REPO = 'Dmitry-dev-pet/alfred-hexapod-lab';
const API = 'https://api.github.com/repos/' + REPO + '/branches/main';
const RAW = 'https://raw.githubusercontent.com/' + REPO + '/';
const ID = /^[a-z0-9-]{1,64}$/;
const LEGACY = /^\/hexapod\/(?:[a-z0-9-]+\/)?$/;
const SHA256 = /^[a-f0-9]{64}$/;

export function modelFor(id) {
  const s = String(id).toLowerCase();
  if (s.startsWith('bb8-')) return 'bb8';
  if (s.startsWith('hexarth-')) return 'hexarth';
  if (s.startsWith('microduck-')) return 'microduck';
  if (s.startsWith('openbeni-')) return 'openbeni';
  if (s.startsWith('en01-')) return 'en01';
  if (s.startsWith('pupper-')) return 'stanford-pupper';
  if (s.startsWith('lite3-')) return 'xgo-lite3';
  if (s.startsWith('xgorider2-')) return 'xgo-rider2';
  if (s.startsWith('wheelbot-')) return 'mini-wheelbot';
  if (s.startsWith('sebaju-')) return 'sebaju';
  if (s.startsWith('cheetah-')) return 'mini-cheetah';
  if (/^(tripod-|claws-|six-to-four|power-loss-|contact-|support-|issue41-|wave-|live-v|freenove-|physics-replay-)/.test(s)) return 'freenove';
  return 'other';
}
function expect(ok, message) { if (!ok) throw Error('Robot library: ' + message); }
function validateLegacyEntry(entry) {
  expect(entry && ID.test(entry.id) && typeof entry.title === 'string' &&
    typeof entry.detail === 'string' && Number.isFinite(entry.duration_s) &&
    entry.duration_s > 0 && LEGACY.test(entry.href) && Array.isArray(entry.files),
    'invalid legacy entry');
  for (const file of entry.files) {
    expect(typeof file.path === 'string' && file.path.startsWith('hexapod/') &&
      !file.path.includes('..') && Number.isSafeInteger(file.size) &&
      file.size >= 0 && SHA256.test(file.sha256), 'invalid legacy file manifest');
  }
}
export function mergeCatalogs(legacy, replay, siteOnly) {
  expect(legacy?.schema === 'hexapod-library/v1' &&
    Number.isInteger(legacy.revision) && Array.isArray(legacy.entries),
    'invalid legacy catalog');
  expect(replay?.schema === 'robot-replay-catalog/v1' &&
    Number.isInteger(replay.revision) && Array.isArray(replay.entries),
    'invalid replay catalog');
  expect(siteOnly?.schema === 'robot-library-site-only/v1' &&
    Number.isInteger(siteOnly.revision) && Array.isArray(siteOnly.entries) &&
    Array.isArray(siteOnly.unpublished_legacy_ids),
    'invalid Site-only inventory');
  const ids = new Set(), hrefs = new Set();
  const unpublished = new Set(siteOnly.unpublished_legacy_ids);
  const entries = [];
  const add = entry => {
    expect(!ids.has(entry.id) && !hrefs.has(entry.href), 'duplicate ID or route: ' + entry.id);
    ids.add(entry.id); hrefs.add(entry.href); entries.push(entry);
  };
  for (const [source, items] of [['legacy-main', legacy.entries], ['legacy-site', siteOnly.entries]]) {
    for (const entry of items) {
      validateLegacyEntry(entry);
      add({...entry, format:'legacy-html', model:modelFor(entry.id),
        source, site_status:unpublished.has(entry.id)?'not-published':'listed'});
    }
  }
  for (const entry of replay.entries) {
    expect(ID.test(entry.id) && typeof entry.title === 'string' &&
      typeof entry.detail === 'string' && Number.isFinite(entry.duration_s) &&
      entry.duration_s > 0 && entry.recording?.schema === 'robot-replay/v1' &&
      SHA256.test(entry.recording.sha256) &&
      Number.isSafeInteger(entry.recording.size) && entry.recording.size > 0 &&
      /^https:\/\/[^/]+\/.+/.test(entry.recording.url),
      'invalid replay entry');
    add({...entry, href:'/replay/?recording=' + entry.id,
      format:'robot-replay/v1',model:modelFor(entry.id),source:'replay-main',
      site_status:'listed'});
  }
  for (const id of unpublished) {
    expect(ids.has(id), 'missing flagged unpublished legacy entry: ' + id);
  }
  return {
    schema:'alfred-robot-library/v1',
    revision:[legacy.revision,replay.revision,siteOnly.revision].join('.'),
    counts:{legacy:legacy.entries.length, site_only:siteOnly.entries.length,
      replay:replay.entries.length,total:entries.length},
    entries
  };
}
export async function readUnifiedLibrary({fetchImpl=fetch}={}) {
  const getJson = async url => {
    const response = await fetchImpl(url,{
      cache:'no-store',credentials:'omit',signal:AbortSignal.timeout(12000)
    });
    if (!response.ok) throw Error('Library HTTP ' + response.status);
    const length = Number(response.headers.get('content-length') || 0);
    if (length > 400000) throw Error('Library document too large');
    const text = await response.text();
    if (text.length > 400000) throw Error('Library document too large');
    return JSON.parse(text);
  };
  const branch = await getJson(API);
  const sha = branch?.commit?.sha;
  expect(typeof sha === 'string' && /^[a-f0-9]{40}$/.test(sha), 'invalid main SHA');
  const names = [
    'docs/hexapod/library.json',
    'docs/replay/catalog.json',
    'docs/robot-library/site-only-legacy.json'
  ];
  const [legacy,replay,siteOnly] = await Promise.all(
    names.map(path => getJson(RAW + sha + '/' + path))
  );
  return {...mergeCatalogs(legacy,replay,siteOnly),source_commit:sha};
}
