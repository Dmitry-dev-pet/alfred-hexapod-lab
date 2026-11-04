# Hexapod Lab — audit готовых контроллеров вместо повторной реализации IK/WBC/MPC

**4 ноября 2026. Решение для Issue #26.** Не переписывать планировщик, IK и оптимизацию ради демонстрации. Сначала проверять готовые реализации на **одной и той же** CAD30-модели и реальном сохранённом состоянии. Не менять `main` и не объединять исследовательские draft PR без согласования.

## Результат аудита

| Проект | Что уже реализовано | Что НЕ решает для нас | Вывод |
|---|---|---|---|
| [Mink 1.4.0](https://github.com/kevinzakka/mink), Apache-2.0 | Дифференциальная IK поверх MuJoCo: frame tasks, joint/velocity limits, collision avoidance, equality constraints, QP | Не гарантирует динамические силы контакта, момент/скорость PWM-сервоприводов, контактный граф/четырёхшаговый горизонт | **Основной кандидат заменить наш локальный кинематический `wbc_contact_correction.py`**, после честного A/B-теста |
| [MuJoCo MPC (MJPC)](https://github.com/google-deepmind/mujoco_mpc) | Физический predictive control и оптимизация траекторий с MuJoCo | Python API экспериментальный и требует совместимого C++-описания задачи; не является готовой четырёхногой FNK0052 походкой | **Отложить интеграцию**, пока не найден кинематически/силово допустимый цикл и не согласована метрика контактов |
| [wk-hexapod](https://github.com/WayneKennedy/wk-hexapod) | Конкретный Freenove FNK0052: Raspberry Pi 5, ROS2, аналитическая IK, штатная шестиногая tripod-походка, PWM PCA9685 | Не готовое доказанное управление «четыре несущие, две подняты, lookahead + micro WBC» | Взять как **инженерный референс** серво-нумерации/калибровки/базовой походки, не копировать вслепую и не навязывать ROS2 |
| [neuroWalknet](https://github.com/hcruse/neuro_walknet) | Исследовательская локальная координация ног | Другой физический стек, не plug-and-play; не равно MPC | Отдельный исследовательский эталон *после* работоспособного кинематического базиса |

Источник Mink: официальные [docs](https://kevinzakka.github.io/mink/index.html), [API FrameTask](https://kevinzakka.github.io/mink/api/tasks.html), [PyPI 1.4.0](https://pypi.org/project/mink/1.4.0/). Источник MJPC: [официальный README, включая ограничения Python API](https://github.com/google-deepmind/mujoco_mpc/blob/main/README.md). Источник FNK0052: [документация Freenove](https://docs.freenove.com/projects/fnk0052/en/latest/fnk0052/codes/tutorial/4_Hexapod_Robot.html).

**Важная поправка к нашей терминологии:** `wbc_contact_correction.py` до сих пор только линейная *кинематическая* компенсация якобианами; называть её полноценным динамическим whole-body controller нельзя. Mink решает похожую *кинематическую* задачу более универсально. Ни Mink, ни наш локальный модуль не гарантируют, что обычный PWM-серво способен выполнить рассчитанный силовой профиль. Для реального робота требуется калибровка и проверка ограничений команд.

## Что сделано в этой ветке: реальная CAD30 подготовка к сравнению

`mink_cad30_probe.py` не создаёт собственный новый IK. Он принимает локальный оригинальный XML и архив qpos, извлекает **фактические** тибиа-пол контакты из позы B при t=9.3 c, добавляет к сохранённым четырём голеням четыре безмассовых `site` (производная модель собирается только в памяти через `MjSpec`) и передаёт эти именованные точки в Mink как `FrameTask`, когда библиотека доступна. Оригинальные STL и XML не изменены.

- Исходное CAD30: 27 qpos, 26 nv, 20 приводов, 30 мешей, MuJoCo 3.14.0.
- Модельные массы и инерции при добавлении четырёх marker-site **совпадают с исходными**.
- Максимальная разница координат marker-site и ранее измеренной физической точки контакта: **9.88e-17 м**.
- По-прежнему два сустава `motor31` и `motor27` нарушают принятый *исследовательский*, НЕ паспортный, предел ±1.7 рад: +1.70841061 и −1.70845257 рад.
- 5 локальных тестов подготовительного слоя на реальном CAD30 **PASS**. Они проверяют отсутствие подмены модели и обязательный честный статус при отсутствии Mink, но **не скорость/качество Mink**.

**Блокер установки:** в доступной вычислительной среде есть MuJoCo 3.14.0 (восстановлен офлайн), но нет Mink/qpsolvers/daqp. Сетевой `pip` недоступен. Поэтому `mink` возвращает `UNAVAILABLE`, `kinematic_gate_pass=false` и `hardware_approved=false`. **Mink не исполнялся**. Сравнения скорости/точности с собственным WBC нет. Скрипт содержит адаптер к документированному API Mink 1.4.0, но его API-совместимость также требует отдельного теста после установки зависимости.

Предыдущий локальный результат WBC из [draft PR #29](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/29): максимальный кинематический дрейф опоры **0.000865 мм** на одном малом движении корпуса, с двумя малыми поворотами и ограничением Δq. Это **исторический ориентир**, а не сравнение одинаковых задач или времени с Mink: в новом пробнике задаётся только трансляция корпуса. Честный A/B-тест требует идентичных целевых поз, регуляционных весов, суставных ограничений и начальных состояний — и измерения *реальных* ошибок через `mj_forward`, не только QP-residual.

## План одного небольшого A/B-теста вместо нового контроллера

1. На Linux CPython 3.13 установить зафиксированный стек из `requirements_mink_audit.txt`; не заменять им проектное окружение и не передавать управление реальным роботом.
2. Выполнить на исходной позе B — с **только исследовательскими, не аппаратными** ограничениями — Mink `FrameTask` для тех же четырёх контактных `site` и перемещения корпуса на 0.35 мм. Заморозить свободные передние ноги и голову; прочитать независимое положение маркеров после `mj_forward`. Скрипт отвергает слишком большой дрейф, несовпадение движения корпуса и увеличение угла >0.02 рад.
3. Сверить обе реализации на одинаковых **измеряемых задачах**, а не просто сравнить заявленное значение ошибки из старых отчётов: тело, четыре toe markers, ±1.7 исследовательский limit (при исходной позе B сначала найти полностью допустимую стартовую позу), разные Δ корпуса, одинаковые допуски, несколько повторов и отдельно runtime P50/P95.
4. Только при реальном PASS/Mink использовать его в новых экспериментах. Силовой контактный LP/QP и контроль фактических сил оставить независимым тестом; проверку настоящего contact MPC отложить.

**Запрет на преждевременную интерпретацию:** успешная IK не доказывает достижимость физического перехода, положительную реакцию каждой ноги, отсутствие slip, текущую доступность заводского серво-диапазона или готовность четырёхногой походки. Не писать новые ходовые контроллеры до этого сравнения.

## Воспроизведение

Не коммитить исходные Freenove STL; локально восстановить проект и state archive с Drive:

- Модель: [CAD30 experiments ZIP](https://drive.google.com/file/d/1qN8KAgekhXm5RasMXSHNxHAsMxivpZ44/view).
- Поза B: [FNK0052 claw-tuck ABC archive](https://drive.google.com/file/d/15mcLT9KHRrxAnjSG7KTTCc02K6aBDJb1/view).

```bash
# зависимости: внешняя сеть / заранее скачанные PyPI wheels
python -m pip install -r experiments/cad30/requirements_mink_audit.txt

python experiments/cad30/mink_cad30_probe.py \
  --model source/mjcf/scene_local30_nominal.xml \
  --pose-zip /path/to/FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip \
  --output results/mink_cad30_probe_local.json

# 5 readiness-тестов; при отсутствии приватных архивов — SKIP, не PASS
CAD30_MODEL_XML="$PWD/source/mjcf/scene_local30_nominal.xml" \
CAD30_REPLAY_ZIP="/path/to/FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip" \
PYTHONPATH=experiments/cad30 python -m unittest -v test_mink_cad30_probe
```

Скрипт не посылает команды приводам; все проверки в MuJoCo — чисто кинематические. `main` остаётся неизменным, это **draft** для ревью.

## Update 4 November 2026 — Mink now actually installed and executed

Previous "upstream unavailable" status below is preserved as a historical first audit result, **but has now been superseded by the executed run**.

- Exact offline dependency bundle: [Google Drive — Mink cp313 Linux x86-64 wheelhouse](https://drive.google.com/file/d/1hnDaarQNdj5_Y94BJ9aNA4j2xRQ-kowv/view?usp=drivesdk), ZIP SHA256 `13ceedb3ad570d65bf53a4d14b258efeb8243e286ffc5c6f015501c000d2d93d`, size 85,237,243 bytes. It contains 14 verified official wheels plus SHA256SUMS and manifest. Not suitable for macOS ARM64 execution.
- [GitHub Actions run #41913088085](https://github.com/Dmytry-lab/runner-smoke/actions/runs/41913088085): `Dmytry-lab/runner-smoke` isolated self-hosted Mac M4 downloaded cross-platform Linux wheels with `pip download`, then Ubuntu Python 3.13 installed them with `pip --no-index` and successfully imported Mink 1.4.0, QPSolvers 4.13.0, DAQP 0.11.0 and MuJoCo 3.14.0. **No numerical robot simulation occurred in GitHub Actions.**
- The same Drive-stored ZIP was fetched back and its archive SHA-256 independently checked; package installed offline into an isolated Linux venv in the ChatGPT compute container.
- On real CAD30 archived pose B, **Mink + DAQP actually solved** the small kinematic correction. Requested base motion (0,+0.350,-0.100) mm, achieved nearly exact. Largest anchored stance-toe marker drift after `mj_forward`: **0.000657 mm**. Largest hinge increment: **0.002188 rad**. Kinematic gate PASS; hardware approval FALSE.
- Previous custom WBC example gave 0.000865 mm drift and 0.002767 rad max increment but also requested additional small rotations and uses different costs. These are **not a controlled apples-to-apples speed/accuracy comparison**.
- Six local test results: existing readiness suite 4 PASS + 1 SKIP (old "not installed" test deliberately skipped now); new **real Mink solver integration test 1 PASS**. Latest machine-readable record: `results/mink_cad30_installed_2026-11-04.json`; new test: `experiments/cad30/test_mink_installed_integration.py`.
- The source pose B still has two joints slightly outside the provisional |q|≤1.7 rad research IK constraint; physical servo limits and dynamic stability remain unknown. Mink's success is geometric only, NOT real four-legged walking.

**Next**: put Mink and the custom WBC under exactly matched pose objectives/limits, then use Mink as a candidate IK module while separately retaining wrench/torque/contact MPC constraints.
