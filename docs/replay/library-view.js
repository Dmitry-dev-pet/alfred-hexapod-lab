// Library presentation only. Recordings and physics stay in their existing readers.
const normalize = value => String(value).normalize('NFKC').toLocaleLowerCase('ru').replaceAll('ё', 'е');

export function filterRecordings(entries, {query = '', robot = ''} = {}) {
  const words = normalize(query).trim().split(/\s+/).filter(Boolean);
  return entries.filter(entry => (!robot || entry.robot === robot) &&
    words.every(word => normalize([entry.robot, entry.title, entry.detail, entry.id].join(' ')).includes(word)));
}

const duration = seconds => seconds < 60 ? `${seconds} с` : `${Math.floor(seconds / 60)} мин ${Math.round(seconds % 60)} с`;

export function createLibraryView({document, onOpen, onRefresh}) {
  const $ = id => document.getElementById(id);
  let entries = [], selected = '', source = '', refreshing = false;
  const expanded = new Set();

  function render() {
    const query = $('search').value, robot = $('robot-filter').value;
    const filtered = filterRecordings(entries, {query, robot});
    const families = new Map();
    for (const entry of filtered) {
      if (!families.has(entry.robot)) families.set(entry.robot, []);
      families.get(entry.robot).push(entry);
    }
    const blocks = [...families].sort(([a], [b]) => a.localeCompare(b, 'ru')).map(([name, items]) => {
      const section = document.createElement('details');
      section.className = 'robot-family';
      section.open = !!query.trim() || !!robot || expanded.has(name) || items.some(e => e.id === selected);
      const summary = document.createElement('summary');
      const title = document.createElement('span'); title.textContent = name;
      const count = document.createElement('span'); count.className = 'family-count'; count.textContent = String(items.length);
      summary.append(title, count); section.append(summary);
      section.addEventListener('toggle', () => {
        if (section.open) expanded.add(name); else expanded.delete(name);
      });
      const list = document.createElement('div'); list.className = 'family-recordings';
      for (const entry of items) {
        const available = entry.availability === 'published';
        const card = document.createElement(available ? 'a' : 'div');
        card.className = 'recording-link' + (available ? '' : ' pending');
        if (available) {
          card.href = entry.href;
          if (entry.kind === 'recording') card.addEventListener('click', event => {
            if (event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
            event.preventDefault(); onOpen(entry);
          });
        } else card.setAttribute('aria-disabled', 'true');
        if (entry.id === selected) card.setAttribute('aria-current', 'true');
        const heading = document.createElement('strong'); heading.textContent = entry.title;
        const detail = document.createElement('span'); detail.className = 'recording-detail'; detail.textContent = entry.detail;
        const meta = document.createElement('span'); meta.className = 'recording-meta';
        // Availability and experiment outcome are separate facts.
        meta.textContent = available ? duration(entry.duration_s) +
          (entry.result_status === 'failed' ? ' · неудачный опыт' : '') : 'Ожидает публикации';
        card.append(heading, detail, meta); list.append(card);
      }
      section.append(list); return section;
    });
    $('recordings').replaceChildren(...blocks);
    $('no-results').hidden = filtered.length !== 0 || entries.length === 0;
    $('clear-search').hidden = !query && !robot;
    $('search-count').textContent = query || robot ? `Найдено: ${filtered.length} из ${entries.length}` : '';
    const ready = entries.filter(entry => entry.availability === 'published').length;
    $('library-count').textContent = `Роботов: ${new Set(entries.map(entry => entry.robot)).size} · карточек: ${entries.length}`;
    $('catalog-status').textContent = `Доступно: ${ready} · ожидают публикации: ${entries.length - ready} · ` +
      (source.startsWith('github-main@') ? 'Обновлено из GitHub' : 'Сохранённая библиотека');
    $('catalog-status').title = source;
  }

  $('search').addEventListener('input', render);
  $('robot-filter').addEventListener('change', render);
  $('clear-search').addEventListener('click', () => {
    $('search').value = ''; $('robot-filter').value = ''; render(); $('search').focus();
  });
  $('refresh-library').addEventListener('click', async () => {
    if (refreshing) return;
    refreshing = true; $('refresh-library').disabled = true;
    $('catalog-status').textContent = 'Проверяем обновления…';
    try { await onRefresh(); } catch (error) { showError(error); }
    finally { refreshing = false; $('refresh-library').disabled = false; }
  });

  function showError(error) {
    $('catalog-status').textContent = 'Не удалось обновить библиотеку: ' + error.message;
  }

  return {
    update(doc) {
      entries = doc.entries; source = doc.source;
      const previous = $('robot-filter').value;
      const all = document.createElement('option'); all.value = ''; all.textContent = 'Все роботы';
      const options = [...new Set(entries.map(entry => entry.robot))].sort((a, b) => a.localeCompare(b, 'ru')).map(name => {
        const option = document.createElement('option'); option.value = name; option.textContent = name; return option;
      });
      $('robot-filter').replaceChildren(all, ...options);
      $('robot-filter').value = options.some(option => option.value === previous) ? previous : '';
      document.documentElement.dataset.librarySource = source; render();
    },
    select(id) { selected = id || ''; render(); },
    showError
  };
}
