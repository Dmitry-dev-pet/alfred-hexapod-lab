  refresh.addEventListener('click', async () => {
    refresh.disabled = true;
    message.textContent = 'Проверяем обновления…';
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 12000);
    try {
      const response = await fetch('/hosted-version.json', {cache:'no-store', signal:controller.signal});
      if (!response.ok) throw Error('Не удалось проверить обновления');
      const version = await response.json();
      if (version.build_id !== HOSTED_BUILD_ID) {
        location.reload();
        return;
      }
      if (!await readLibrary(controller.signal)) throw Error('Не удалось загрузить каталог');
      draw();
      message.textContent = 'Библиотека актуальна · записей: ' + entries.length;
    } catch (error) {
      message.textContent = error.name === 'AbortError'
        ? 'Сервер не ответил. Проверьте соединение и повторите.'
        : 'Не удалось проверить обновления. Повторите при подключении к интернету.';
    } finally {
      clearTimeout(timer);
      refresh.disabled = false;
    }
  });
