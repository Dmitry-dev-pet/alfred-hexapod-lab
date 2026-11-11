# Hexapod Lab — интерактивный просмотрщик 60 секунд

**Записанная физика, не живое управление в браузере.** `data/tripod-60s.json` содержит 1501 кадр положения корпуса и геометрических ориентиров ног, полученных из 60-секундного **непрерывного** расчёта MuJoCo 3.14.0 с 30 из 32 оригинальных CAD-сеток в приватном стенде. Реальные STL **не включены** в публичные файлы. В Three.js используется нарисованная нами схематическая механическая оболочка, а не копия Freenove CAD.

## Открыть

Если GitHub Pages настроен на ветку с этим каталогом и источник `/docs`, начальная страница `/docs/index.html` автоматически направляет на `/hexapod/`.

Локально: `python -m http.server 8000 -d docs`, затем `http://localhost:8000/hexapod/` (для Three.js требуется доступ к CDN).

## Проверенный опыт

- `MuJoCo 3.14.0`, Python 3.13.5, `Freenove FNK0052`, CAD30 nominal, 1.804868 кг по модельным оценкам.
- Один опыт tripod продолжительностью **60.000 с** после 1-секундной стабилизации; **30 000** физических интеграционных шагов при `dt=0.002 c`.
- Расчётное продвижение вдоль оси CAD — **1.102688 м**, максимальный наклон **0.358°**, минимум **3** фактически контактирующие ноги, максимальный модельный момент **0.39151 Н·м** при *условном* ограничении ±0.7 Н·м, без насыщений и падения по критериям стенда.
- Шаг команды 35 мм, подъём 20 мм, период 2.4 с; установленные физические параметры **не измерялись на железе**.
- `qpos` из исходного расчёта фиксировался на 25 Гц. Точки шарниров и геометрический ориентир стопы воспроизведены через `mj_forward` и `xanchor` / `tip_world`, а опорные состояния получены из отдельных 50-Гц записанных контактов.
- Геометрический tip — дальняя вершина CAD-сетки относительно коленного шарнира в нулевой позе. Он **не** гарантированно совпадает с реальной точкой контакта резиновой подошвы.
- Доказано только отсутствие падения и обрывов контакта менее трёх ног в **этом** 60-секундном модельном проходе; не сертифицирована статическая устойчивость на каждом отсчёте и не учтены реальные люфты, ток, нагрев и скорость серв.

## Повторить экспорт данных

На локальной машине с MuJoCo 3.14.0, NumPy, SciPy и *приватными* закреплёнными CAD активами:

```bash
python locomotion_cad30.py --gait tripod --mass nominal --seconds 60
PYTHONPATH=. python scripts/export_viewer_trajectory.py \
  --source results/cad30_gait_tripod_nominal.json \
  --model source/mjcf/scene_local30_nominal.xml \
  --out docs/hexapod/data/tripod-60s.json
```

Пути выше относительно восстановленного рабочего стенда: скрипт экспорта хранится в репозитории в `scripts/`. Тяжёлые исходники и полный `qpos` receipt — в приватной папке проекта на Google Drive, контрольные суммы в протоколе.

## Публикация

Статический GitHub Pages: **Settings → Pages → Deploy from branch → нужная ветка /docs**. Никакого backend и GitHub Actions для вычислений не требуется. Если этот код остаётся в draft PR, сайт не появляется на Pages автоматически. При переносе в `main` не размещать оригинальные `.stl`, `.xml` и архивы CAD в `/docs`.

JavaScript использует CDN Three.js 0.169.0 (`three.module.js` и `OrbitControls.js`) через import map. Мобильные касания поддерживаются; на GitHub Pages нужны HTTPS и доступ к CDN.

## CAD viewer update — 4 November 2026

The published site is hosted at **https://hexapod-lab-viewer.pages.dev/**, served by Cloudflare Pages from branch `feature/threejs-60s-physical-replay-20261104`, directory `docs/`; PR #20 remains draft.

The default **Original CAD** mode now renders the authentic public Freenove mesh shapes (`30/32` STL assets) directly from the pinned public MattiaGrigoli revision `5c9a84c16335b30036e451c691d2c544abf4bc72`. The site **does not host copies** of the original STL files: assets are loaded on the client from their upstream `raw.githubusercontent.com` URLs. The missing `topplate.stl` and `bigservo1.stl` remain the **19 visual-only box substitutes** inherited from the CAD30 simulation. The other added cylinder/box objects represent estimated batteries/electronics. Attribution: Freenove owns the CAD; Mattia Grigoli's [third-party notice](https://github.com/MattiaGrigoli/autonomous_hexapod/blob/main/LICENSE-3RD-PARTY.md) documents his permission to publish the originals for open-source ROS2 simulation. His separate repository MIT license does not cover Freenove CAD. This is a noncommercial academic demonstration; consult underlying rights before republishing mesh binaries.

`data/cad-rig-60s.json` contains **27 physical bodies × 1501 recorded MuJoCo poses** and **146 source-model visual geometry references**, derived by running `mj_forward` on the 60 s saved physical `qpos` records. Each rigid-body pose is read from MuJoCo; transforms are *not* inferred from the schematic six-leg points. Source asset mesh-centering/orientation (`mesh_pos`, `mesh_quat`) is reversed before applying the original MJCF visual-geom transforms. Tests verify the transformed STL vertex positions to `1e-7 m` against MuJoCo's compiled mesh data and sample poses to `1e-6`.

A checkbox can switch back to the schematic view. The site shows a live count `CAD: N / 30 STL` while assets load. If the upstream asset server fails or a WebGL problem prevents loading, the schematic replay remains the fallback. Please test 3D visual frame rate on Android; no new physical simulation was run for this visual update.

Recreate from private source CAD30 and original 60 s recording (kept on Google Drive):

```bash
python scripts/export_cad_rig_replay.py --model source/mjcf/scene_local30_nominal.xml \
  --recording HexapodLab_Tripod_60s_2026-11-04_no_CAD.zip \
  --output docs/hexapod/data/cad-rig-60s.json
python -m unittest discover -s tests -v
```

Publication is intentionally a live **replay** of known MuJoCo physics, not an in-browser physics engine; original STL source remains upstream. Browsers require HTTPS or a local HTTP server for ES modules and trajectory files.