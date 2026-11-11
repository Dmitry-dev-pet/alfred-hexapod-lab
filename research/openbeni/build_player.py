"""Specialize the existing measured multi-record viewer for the OpenBeni rig."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"docs/hexapod/en01-getup-v7"
DEST=ROOT/"docs/hexapod/openbeni-template"
DEST.mkdir(parents=True,exist_ok=True)
app=(SOURCE/"app.js").read_text()
app=app.replace("// Measured nine-body states from the source four-bar adapter; no simulation here.","// OpenBeni adapter of the existing EN01 measured-record viewer; eight original meshes, no physics in this player.")
app="const REFERENCE_REPLAY='/hexapod/data/openbeni-template.json';\n"+app
app=app.replace("'/'+index.rig.cad_asset","index.rig.cad_asset")
app=app.replace("cad.part_count!==73||cad.parts?.length!==73||cad.source?.git_blob_sha1!==index.step_git_blob_sha1", "cad.part_count!==8||cad.parts?.length!==8||cad.source?.commit!==index.source_commit")
app=app.replace("document.documentElement.dataset.cadParts='73'", "document.documentElement.dataset.cadParts=String(cad.part_count)")
app=app.replace("time.toFixed(1)+' / 6,0 с'", "time.toFixed(1)+' / '+data.duration_s.toFixed(1)+' с'")
start=app.index("  $('pose').textContent=")
end=app.index("\n}",start)
app=app[:start]+"""  const m=data.metric_start;
  $('pose').textContent='Наклон '+a[m].toFixed(1)+'° · скорость '+a[m+1].toFixed(3)+' м/с';
  $('contact').textContent='Колёса: '+a[m+3].toFixed(2)+' / '+a[m+4].toFixed(2)+' Н · корпус '+a[m+2].toFixed(2)+' Н';
  $('joints').textContent=a[m+5]?'Опора на оба колеса':'Переходная поза';"""+app[end:]
start=app.index("    $('verdict').textContent=")
end=app.index("    document.documentElement.dataset.replayClip",start)
app=app[:start]+"""    $('verdict').textContent=m.recovered_1s?'Подъём в исходной модели получен':'Подъём не получен';$('verdict').dataset.status='research';
    $('metrics').textContent='Спокойная стойка в конце: '+(m.final_2s_quiet?'да':'нет')+' · наклон '+m.final.tilt_deg.toFixed(1)+'°';
    $('scenario').textContent=clip.label;
    $('result-details').textContent='Начало полной секунды опоры: '+(m.recovery_onset_s===null?'нет':m.recovery_onset_s.toFixed(2)+' с после включения')+'. Максимальное проникновение в пол: '+(m.peak_floor_penetration_m*1000).toFixed(2)+' мм. Это результат упрощённой контактной модели.';
    $('provenance').textContent='OpenBeni 4545b86a · SHA-256 трассы: '+clip.trace_sha256+' · '+next.frame_count+' измеренных кадров.';
    seek.max=String(next.duration_s);
"""+app[end:]
app=app.replace("'Не удалось открыть EN01: '","'Не удалось открыть OpenBeni: '")
app=app.replace("if(time>6)time=0", "if(time>data.duration_s)time=0")
app=app.replace("index=validateIndex(await r.json());await robot();", "index=validateIndex(await r.json());const ref=index.clips.find(c=>c.id===index.default_clip);await loadVerified(REFERENCE_REPLAY,index.reference.size,index.reference.sha256,b=>decodeRecord(b,ref,index));await robot();")
(DEST/"app.js").write_text(app)
(DEST/"replay.css").write_text((SOURCE/"replay.css").read_text())
html=(SOURCE/"index.html").read_text()
html=html.replace("EN01 · вставание после падения · v7","OpenBeni · проверка вставания")
html=html.replace("EN01: 20 проверок вставания после падения","OpenBeni: измеренные проверки вставания")
html=html.replace("Оригинальная модель EN01","Исходная модель OpenBeni").replace("Запись EN01 v7","Запись OpenBeni")
start=html.index('<p id="mode">');end=html.index('<p id="provenance">',start)
html=html[:start]+'''<p id="mode">17 записей: авторские настройки контактов, отдельная проверка с более жёстким контактом и пассивные контроли.</p>
<p id="result-details"></p><p>Первые 0,5 с лежачих запусков — свободное успокоение; затем работает готовая авторская нейросеть. Внешних толчков и сбросов позы нет.</p>
<p>Визуальные меши исходные; контактные оболочки автора — коробки и цилиндры. В исходной серии пол проникается до 20,5 мм; при времени контакта 5 мс — до 3,3 мм. Геометрическая и аппаратная достоверность не подтверждена.</p>'''+html[end:]
html=html.replace('Четыре исходные лежачие позы · пять вариантов команд.','Стойка, живот, спина, оба бока и перевёрнутое положение.')
html=html.replace('<script src="/github-live-boot.js"></script><script src="/webgl-fallback.js"></script>','')
html=html.replace('OpenBeni · проверка вставания','OpenBeni · проверка вставания · openbeni-template')
(DEST/"index.html").write_text(html)
(DEST/"record-loader.js").write_text((Path(__file__).parent/"player-template/record-loader.js").read_text())
print(DEST)
