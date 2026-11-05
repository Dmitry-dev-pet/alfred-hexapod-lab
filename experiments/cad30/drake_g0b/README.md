# CAD30 G0b — Drake ↔ MuJoCo: реальные контакты и приводы (2026-11-05)

Связи: [Issue #50](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/50), PR #57 (рабочий IDTO + finite-box floor), протокол периодической походки PR #53. **Исследование выполнено**, но полная физическая/силовая эквивалентность **НЕ пройдена**, полноценной четырёхногой походки **НЕТ**. Это не запуск IDTO с исправленными приводами.

## Результат

Использованы сохранённые физические состояния оригинального Freenove CAD30 из Pupper PR #40: (A) 6 опор после предбалансировки, (B) 4 опоры и уже физически поставленная BL. Исходный MuJoCo 3.14 MJCF с бесконечной плоскостью; Drake 1.30 копия той же модели с конечным `box` вместо пола (PR #57), при неизменных шести tibia collision mesh. Исходная модель, оригинальные STL и qpos/qvel принадлежат владельцу, **не коммитятся**.

### G0b.1 — Геометрия, инерция и параметры приводов

Проверены MuJoCo→Drake отображения свободного тела (порядок `q` и ориентация `v`), 27 позиций, 26 скоростей, 20 actuators. В *двух сохранённых состояниях* максимальная разница signed-distance ground-to-tibia между Drake и MuJoCo для общей finite-box сцены — менее **1.5×10⁻⁸ м**. Это сравнение **самой глубокой точки каждой tibia**; Drake даёт один closest-pair на tibia, MuJoCo может дать два контактных пункта. Нет утверждения о полной эквивалентности manifolds.

Обнаруженные в PR #57 недостатки импортированного Drake MJCF:
- У **18 leg actuators** `effort_limit=∞`, хотя MuJoCo XML задаёт исследовательскую границу `±0.7 Н·м`; два head actuators имели `±0.2 Н·м`.
- В MuJoCo для каждого из 20 hinge DOFs есть `armature=0.002 кг·м²`, но Drake Parser его пропускает; максимальное расхождение диагоналей масс ~0.002.
- Вязкое демпфирование `0.035` соответствует, а MuJoCo сухое `frictionloss=0.015` не перенесено в Drake.

Проверенный **локальный, не включённый в IDTO baseline** способ исправления: удалить и добавить joint actuators обратно в том же порядке с `effort_limit=0.7`/`0.2`, затем в Drake `set_default_rotor_inertia(0.002)` и `gear_ratio=1`. Так 20/20 моторных metadata limits и инерции восстановлены; максимальная разница диагоналей масс в обоих сохранённых состояниях **~2.22×10⁻¹⁶**. **Внимание:** `effort_limit()` в Drake MultibodyPlant — *метаданные*, а не физическое насыщение входа; ограничение усилий необходимо явно включать в IDTO/контроллер. Роторную инерцию восстановили; сухое трение пока нет.

### G0b.2 — один реальный 2-мс шаг динамики

На **одинаковых физических начальных `qpos/qvel`** обе системы получили одинаковый вектор управляющих моментов (позиционный PD `Kp=7`, `Kd=0.32`, cap ±0.7 Н·м). Drake `MultibodyPlant(time_step=0.002)` настроен с исправленными inertial actuator parameters; MuJoCo получил их из исходного XML. Для контроля MuJoCo запущен и на оригинальной плоскости, и на конечном box-полу.

| Состояние | MuJoCo исходный plane, ΣFz | MuJoCo конечный box, ΣFz | Drake box, ΣFz |
|---|---:|---:|---:|
| 6 loaded | 17.705814 Н | 17.559902 Н | 2.754710 Н |
| 4 loaded | 17.705577 Н | 17.274334 Н | 2.496364 Н |

Drake `ContactResults::PointPairContactInfo` зарегистрировал **6** и **4** реакций соответственно. Это настоящие выходы одномоментного 2-мс шага Drake, а не IDTO-optimal force или статическая LP.

**FAIL контактов:** разница порядка 15 Н показывает, что default *compliant point contact* Drake 1.30 не повторяет MuJoCo `solref/solimp` с глубинами микрометров. Также конечный box уже меняет число контактных пунктов и силы *внутри MuJoCo*. Отождествление отдельных контактных пар и строгой суммарной реакции без настройки физики некорректно. МуJoCo `mj_step` предоставляет силы на начале шага, Drake результат относится к прошедшему шагу: это дополнительный источник расхождений, но не основание объявлять parity.

Номинальная сила тяжести ~17.706 Н. Сильный разрыв показывает, что нельзя сейчас утверждать физическую работоспособность оптимизированных IDTO траекторий только по выходу Drake. **Контактная динамика G0b = FAIL**; не утверждаем общий запрет походки.

## Реальные тесты и воспроизводимость

- `drake_contacts_probe.cc` — независимый C++/Drake 1.30 `Diagram + SceneGraph` с geometry signed-distance, 20 actuator checks, optional corrected rotor inertia and actuator limits, и настоящим дискретным `2ms` step + contact forces.
- `g0b_contact_audit.py` — восстановление MuJoCo из приватных данных, вычисление настоящих `mj_contactForce`, сопоставление с записанными Drake outputs, JSON c SHA256, **без** joint-state vectors и mesh.
- `test_g0b_contact_audit.py` — чистые unit tests (4/4 PASS), без приватной геометрии.
- `results/G0B_CONTACT_ACTUATOR_RECEIPT.json` — конкретные числа, в том числе по каждой ноге, статусы PASS/FAIL и SHA256 исходных приватных сцен/snapshots; **машинная трасса не является новой симуляцией**.
- Компиляция: `cmake -S . -B build -Ddrake_DIR=/.../drake/lib/cmake/drake -DCMAKE_PREFIX_PATH="/.../sysroot/usr;/.../drake" -DCMAKE_CXX_COMPILER=clang++`, затем `cmake --build build`.
- Запуск C++: `build/drake_contacts_probe /private/drake_box_scene.xml /private/qv_tau_six.txt --repair-actuators`; файл входа содержит **q в Drake-order, v в Drake-order и u=20 actuator torques**. Эти приватные состояния/модель не публикуются.
- Python: `PYTHONPATH=/path/to/mujoco-wheels python g0b_contact_audit.py --mu-plane /private/mjcf/scene.xml --drake-box /private/drake_box_scene.xml --state-six /private/pupper_stance_prebalance_35mm.json --state-four /private/pupper_four_bl20_simple.json --drake-logs /private/outputs --out results/G0B_CONTACT_ACTUATOR_RECEIPT.json`.
- `python -m unittest -v test_g0b_contact_audit` — 4 PASS.

## Дальнейшие ограничения для решения полного периода

1. Не подменять MuJoCo силу под каждой стопой «модельной» Drake без экспериментальной проверки. Сначала сравнить динамику после нескольких шагов и исследовать жёсткость/податливость, характеристики контакта и сухое трение; отдельные численные схемы не обязаны совпадать побайтно. Цель — совпадение измеримых нагрузок и отсутствия нежелательных отрывов в диапазоне реальных физических состояний.
2. Сохранить исправленные инерции и учёт усилий в **обеих** моделях IDTO (оптимизатор и симуляция); добавить **действительно жёсткий** torque cap, а не только `effort_limit` в метаданных. Иначе `ACTUATOR_PARITY=NOT_PROVEN`.
3. После этого искать целый период по G1 PR #53: все 4 swing, замыкание поз/скоростей, 3+ реальные опоры и положительный `net Y` *с подготовкой*; не возвращаться к ручному подбору одиночной ноги.
4. Только тогда независимый непрерывный G2 MuJoCo ≥3 циклов, и исследовательские параметры не выдавать за аппаратно проверенные.

**Текущий статус:** `G0b_GEOMETRY_PASS / G0b_ACTUATOR_INERTIA_METADATA_PASS / G0b_CONTACT_FORCE_FAIL / CORRECTED_IDTO_NOT_RUN / GAIT_NOT_FOUND`.

Не запускать реальное железо; не включать GitHub Actions для физики; не сливать в `main` без разрешения владельца. Полные служебные логи доступны в приватном архиве Drive с SHA256.
