const status = document.querySelector('#status');
try {
  const response = await fetch('./media.json', {cache:'no-store'});
  if (!response.ok) throw Error('Не удалось загрузить каталог');
  const data = await response.json();
  for (const item of data.videos) {
    if (!/^[a-zA-Z0-9_-]+\.mp4$/.test(item.filename)) continue;
    const card = document.createElement('article'); card.className='video-card'; card.id=item.filename;
    const video = document.createElement('video');
    video.controls=true; video.playsInline=true; video.preload='metadata';
    video.src='./'+item.filename; video.setAttribute('aria-label',item.title);
    const info = document.createElement('div'); info.className='video-info';
    const title = document.createElement('h2'); title.textContent=item.title;
    const description = document.createElement('p'); description.textContent=item.description;
    const origin = document.createElement('a'); origin.textContent='Материалы эксперимента ↗';
    const url = new URL(item.source_url);
    if (url.protocol === 'https:' && url.hostname === 'drive.google.com') {
      origin.href=url.href; origin.target='_blank'; origin.rel='noopener noreferrer';
      info.append(title,description,origin);
    } else info.append(title,description);
    card.append(video,info); document.querySelector('#videos').append(card);
  }
  status.textContent = data.videos.length ? '' : 'В этой публикации ещё нет видеозаписей.';
  if (location.hash) {
    const target = document.getElementById(decodeURIComponent(location.hash.slice(1)));
    target?.scrollIntoView({block:'start'});
  }
} catch (error) {
  status.textContent='Не удалось загрузить видеозаписи. Обновите страницу при подключении к интернету.';
}
