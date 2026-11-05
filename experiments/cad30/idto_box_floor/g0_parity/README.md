# CAD30 Drake ↔ MuJoCo: честный G0 тест (2026-11-05)

## Зачем

Вместо повторного восстановления CAD30 здесь проверяется прямой импорт нашего **оригинального MJCF** (MuJoCo 3.14) в существующий Drake 1.30 — до любого нового решателя походки. См. [Issue #50](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/50), [IDTO PR #33](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/33), [глобальный протокол P0](../../../docs/periodic_gait/PROTOCOL.md). **Ни одна mesh CAD не загружается в GitHub.**

## Исходники

- `cad30_drake_probe.cc` — независимый C++ Drake MJCF Parser/MultibodyPlant: число координат, масса, COM, шесть tibia origins, порядок суставов/приводов, матрица масс и гравитационные силы.
- `convert_stl_to_obj.py` — **только локально**: точная геометрическая конверсия треугольников 30 приватных STL в OBJ, потому что Drake 1.30 MJCF Parser не поддерживает STL напрямую. Визуальные/контактные свойства **не обязаны** совпасть после конвертации и требуют отдельного gate.
- `compare_parity.py` — независимое сравнение с MuJoCo из **того же оригинального MJCF**. Для freejoint MuJoCo q=(xyz,quat), v=(v_linear_world,omega_body), а Drake q=(quat,xyz), v=(omega_world,v_linear_world). Матрицы сравниваются при `omega_world = R_base * omega_body`, после вычитания `dof_armature` из оригинальной MuJoCo матрицы масс (эта настройка игнорируется Drake Parser 1.30).
- `test_compare_parity.py` — чистые unit smoke-тесты обработки неполных данных.
- `results/cad30_drake_mu_G0_20261105.json` — результаты **реальных** Drake C++ и MuJoCo Python запусков с исходной CAD30 из личного Drive, без исходной CAD.

## Воспроизведение

Потребуются: исходные приватные `source/mjcf/*.xml`, `source/meshes/*.stl` с Google Drive; Drake **1.30.0** C++ SDK, MuJoCo **3.14.0** Python, NumPy и Trimesh. Для компиляции в обычном окружении используйте CMake с установленным `drake_DIR`.

```sh
python convert_stl_to_obj.py --source /private/CAD30/source --destination /private/tmp/cad30-drake
cmake -S . -B build -Ddrake_DIR=/path/to/drake/lib/cmake/drake
cmake --build build -j2
./build/cad30_drake_probe /private/tmp/cad30-drake/mjcf/scene_local30_nominal.xml > /private/tmp/drake.txt 2> /private/tmp/parser_warnings.log
python compare_parity.py \
  --mujoco-xml /private/CAD30/source/mjcf/scene_local30_nominal.xml \
  --drake-probe /private/tmp/drake.txt --out /private/tmp/cad30_drake_mu_G0.json
python -m unittest test_compare_parity.py -v
```

## Проверенный результат

В исходной модели: **27 q, 26 v, 48 bodies, 20 actuators, 1.8048682 kg** в обоих движках. Все 20 имён/порядок приводов и все 20 подвижных револьверных суставов совпадают. Координаты шести tibia имеют абсолютную ошибку порядка 1e-13 m. Гравитационные обобщённые силы после приведения скорости к одной системе — порядка 1e-13 N/Nm; матрица масс после исключения неимпортированного `armature=0.002 kg m²` — порядка 1e-13 (фактические точные значения в JSON receipt). **Это кinematic/inertial parity PASS.**

## Следующий блокер

Оригинальные `STL→OBJ` не являются автоматическим подтверждением контактной эквивалентности: MuJoCo contact mesh convexification и Drake proximity/mesh collision могут отличаться. Кроме того, Drake импортирует суставные актуаторы конечностей с `effort_limit=inf`, вместо исходного модельного ±0.7Nm; игнорирует `joint.armature`, `frictionloss` и часть soft-contact. Нельзя выдавать импорт за «IDTO теперь умеет ходить». До G1 на четырёх ногах нужно явно согласовать 18 лимитов моментов/PD, инерции роторов, контактные радиусы/коэффициенты, проверить одинаковый ground-contact при одной позе, а затем перенести готовый IDTO. Реальное железо остаётся выключенным. Здесь нет gait solver run.

**Статус:** `KINEMATIC_INERTIAL_PARITY_PASS / G0_CONTACT_ACTUATOR_NOT_VERIFIED / IDTO_CAD30_NOT_RUN`. Всё в draft PR; main без изменений.
