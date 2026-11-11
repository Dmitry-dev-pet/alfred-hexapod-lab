// One immutable geometry download per robot. Movement traces stay separate.
// Generated packages retain their original data licence and attribution.
const pending = new Map();
const CACHE = 'alfred-robot-model-packages-v1';
const sha256 = async bytes => Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)),
  b => b.toString(16).padStart(2, '0')).join('');

export function validateReference(ref) {
  if (ref?.schema !== 'robot-model-package-ref/v1' ||
      !/^[a-z0-9-]+$/.test(ref.id) || !/^[0-9a-f]{64}$/.test(ref.sha256) ||
      ref.url !== '/assets/robot-models/'+ref.id+'-'+ref.sha256+'.json' ||
      !Number.isInteger(ref.size) || ref.size < 1 || ref.size > 25_000_000 ||
      !Number.isInteger(ref.mesh_count) || ref.mesh_count < 1 || ref.mesh_count > 180 ||
      !Number.isInteger(ref.instance_count) || ref.instance_count < 1 || ref.instance_count > 500 ||
      !/^[0-9a-f]{40}$/.test(ref.source_revision) ||
      typeof ref.license_spdx !== 'string' || !ref.license_spdx) throw Error('Некорректная ссылка на пакет модели');
  return ref;
}
async function boundedBytes(response, limit) {
  if (!response.ok) throw Error('Пакет модели недоступен: HTTP '+response.status);
  const reader = response.body.getReader(), chunks = []; let total = 0;
  try {
    while (true) {
      const {done, value} = await reader.read(); if (done) break;
      total += value.byteLength;
      if (total > limit) throw Error('Превышен размер данных модели');
      chunks.push(value);
    }
  } catch (e) { await reader.cancel().catch(()=>{}); throw e; }
  const bytes = new Uint8Array(total); let offset = 0;
  for (const chunk of chunks) { bytes.set(chunk, offset); offset += chunk.byteLength; }
  return bytes.buffer;
}
export async function unpackModelPackage(bytes, ref, onProgress = ()=>{}) {
  validateReference(ref);
  if (bytes.byteLength !== ref.size || await sha256(bytes) !== ref.sha256)
    throw Error('Контрольная сумма пакета модели не совпадает');
  const pack = JSON.parse(new TextDecoder().decode(bytes)), manifest = pack.manifest;
  if (pack.schema !== 'robot-model-package/v1' || pack.id !== ref.id ||
      pack.encoding !== 'gzip-stl-base64' || manifest?.revision !== ref.source_revision ||
      !manifest.meshes || !Array.isArray(manifest.visible_geoms) ||
      manifest.visible_geoms.length !== ref.instance_count ||
      Object.keys(manifest.meshes).length !== ref.mesh_count ||
      !pack.meshes || Object.keys(pack.meshes).length !== ref.mesh_count ||
      !pack.license?.notice || pack.license.spdx !== ref.license_spdx)
    throw Error('Повреждена структура пакета модели');
  const meshes = new Map(); let total = 0;
  for (const [name, spec] of Object.entries(manifest.meshes)) {
    if (!/^[A-Za-z0-9_]+$/.test(name) || !Number.isInteger(spec.size) ||
        spec.size < 84 || spec.size > 8_000_000 || !/^[0-9a-f]{64}$/.test(spec.sha256) ||
        typeof pack.meshes[name] !== 'string' || pack.meshes[name].length > 12_000_000)
      throw Error('Неверная деталь пакета модели');
    total += spec.size; if (total > 80_000_000) throw Error('Модель слишком велика');
    const compressed = Uint8Array.from(atob(pack.meshes[name]), c=>c.charCodeAt(0));
    const stream = new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'));
    const original = await boundedBytes(new Response(stream), spec.size);
    if (original.byteLength !== spec.size || await sha256(original) !== spec.sha256)
      throw Error('Контрольная сумма STL не совпадает: '+name);
    meshes.set(name, original); onProgress(meshes.size, ref.mesh_count);
  }
  return {manifest, meshes, license: pack.license};
}
export async function loadModelPackage(ref, {onProgress = ()=>{}} = {}) {
  validateReference(ref);
  if (pending.has(ref.sha256)) return pending.get(ref.sha256);
  const operation = (async()=>{
    const cache = globalThis.caches ? await caches.open(CACHE).catch(()=>null) : null;
    if (cache) {
      const saved = await cache.match(ref.url);
      if (saved) {
        try { return await unpackModelPackage(await boundedBytes(saved, ref.size), ref, onProgress); }
        catch { await cache.delete(ref.url); }
      }
    }
    const bytes = await boundedBytes(await fetch(ref.url, {cache:'no-store'}), ref.size);
    const result = await unpackModelPackage(bytes, ref, onProgress);
    if (cache) await cache.put(ref.url, new Response(bytes, {
      headers:{'Content-Type':'application/json'}
    })).catch(()=>{});
    return result;
  })();
  pending.set(ref.sha256, operation);
  try { return await operation; } catch (e) { pending.delete(ref.sha256); throw e; }
}
