  refresh.addEventListener('click', async () => {
    refresh.disabled = true;
    message.textContent = 'Проверяем новые записи в GitHub…';
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 18000);
    try {
      const ready = await window.alfredGithubCatalogBoot?.();
      if (!ready) throw Error('Сервис загрузки GitHub недоступен');
      const before = entries.length;
      if (!await readLibrary(controller.signal)) throw Error('Не удалось получить каталог');
      if (!['github','cached-github'].includes(lastCatalogSource)) throw Error('Пока доступна только встроенная библиотека');
      draw();
      message.textContent = lastCatalogSource === 'cached-github'
        ? 'Нет связи с GitHub · сохранён проверенный каталог: ' + entries.length
        : entries.length > before
          ? 'Добавлено записей: ' + (entries.length - before) + ' · всего ' + entries.length
          : 'GitHub-библиотека актуальна · записей: ' + entries.length;
    } catch (error) {
      message.textContent = error.name === 'AbortError'
        ? 'GitHub не ответил за 18 секунд. Сохранённые записи доступны.'
        : 'Не удалось обновить GitHub: ' + String(error.message || error) + '. Старые записи сохранены.';
    } finally {
      clearTimeout(timer);
      refresh.disabled = false;
    }
  });
