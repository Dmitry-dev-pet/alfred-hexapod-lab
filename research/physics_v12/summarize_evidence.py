#!/usr/bin/env python3
"""Audit physics records (mj_step) and generate human/machine-readable receipts."""
from __future__ import annotations
from pathlib import Path
import hashlib,json,math,numpy as np
HERE=Path(__file__).resolve().parent
EXPECTED={'nominal_full':29,'upper_mass_full':10,'friction_035_full':7,'torque_035_full':7}

def hashfile(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def bounded(v,nd=3):return round(float(v),nd) if v is not None else None

def main():
    root=HERE.parents[1]
    targets=HERE/'data/targets_v11.json'
    source_scene=root/'source/mjcf/scene_local30_nominal.xml'
    inputs=json.loads(targets.read_text())
    assert len(inputs['cases'])==29
    results={};verified_count=0
    for var,expected in EXPECTED.items():
        rdir=HERE/'results'/var
        files=sorted(x for x in rdir.glob('*.json') if x.name!='summary.json')
        assert len(files)==expected,(var,len(files),expected)
        records={}
        for f in files:
            a=json.loads(f.read_text());assert a['id']==f.stem and a['no_root_pose_injection'] and a['no_foot_welding']
            assert a['model_version']=='3.14.0' and a['completed'] and not a['fall'],(var,f.stem)
            assert a['requested']==inputs['cases'][a['id']]['spec']
            assert a['mass_kg']>1.6 and a['mass_kg']<2.2
            assert a['actuator_saturation_fraction']>=0 and a['actuator_saturation_fraction']<=1
            assert a['torque_cap_leg_Nm']<=.7
            assert a['samples'] and len(a['samples'])>=40
            # Physiological bounds are intentionally NOT claimed: this is only simulation evidence.
            records[a['id']]=a;verified_count+=1
        results[var]=records
    # Core reference source identities and host provenance
    assert hashfile(source_scene)=='fe9d7c292b199f19b11da220e74f826e05fb5433a43c5b46c1bc620eb34f6df3'
    assert hashfile(root/'docs/hexapod/data/freenove-original-v7-kit.json')=='5f88588f17e1a1fcef73e20e0787413bc402fd2c676fff3a9c5463315a7b7d8c'
    # True motor commands expected from JS FactoryPilot; do not infer from any root scene transform.
    def evidence(a):
        rows=a['samples']; active=[r for r in rows if r['t_s']<=a['requested']['active_s']+.01] or rows
        yaw=np.rad2deg(np.unwrap(np.deg2rad([r['yaw_deg'] for r in rows])))
        yaw_active=yaw[len(active)-1]
        est=inputs['cases'][a['id']]['visual_dead_reckoning_end']
        return {'id':a['id'],'gait':a['gait'],'mass_kg':a['mass_kg'],
          'duration_s':a['duration_simulated_s'],'no_fall':a['completed'] and not a['fall'],
          'max_tilt_deg':a['max_tilt_deg'],'min_height_m':a['min_root_height_m'],
          'actual_net_xy_m':a['net_xy_from_settle_m'],
          'visual_odom_net_xy_m':bounded(math.hypot(est['x_m'],est['y_m']),5),
          'actual_yaw_unwrapped_deg':bounded(yaw[-1],3),'actual_yaw_at_active_end_deg':bounded(yaw_active,3),
          'command_odometry_yaw_deg':bounded(est['yaw_deg'],3),
          'max_torque_Nm':a['max_abs_motor_torque_Nm'],'saturation_fraction':a['actuator_saturation_fraction'],
          'min_real_contacts':a['min_true_grounded_legs'],'min_support_margin_mm':a['minimum_signed_contact_support_margin_mm'],
          'max_loaded_contact_slip_speed_mm_s':a['max_load_bearing_contact_tangential_speed_mm_s'],
          'loaded_contact_fraction_speed_gt_5mm_s':a['fraction_loaded_contacts_slipping_gt_5mm_s'],
          'active_fraction_with_lt3_contacts':bounded(sum(r['true_grounded_legs']<3 for r in active)/len(active),4),
          'time_with_no_contacts_s':bounded(sum(r['true_grounded_legs']==0 for r in active)*.02,3)}
    cases={var:{name:evidence(a) for name,a in entries.items()} for var,entries in results.items()}
    verified={'schema':'hexapod-cad30-V11-mujoco-physical-20261106-v12',
      'status':'RESEARCH_EVIDENCE_NOT_HARDWARE_PASS','model_version':'MuJoCo 3.14.0',
      'source_scene_sha256':hashfile(source_scene),'full_controller_trace_sha256':hashfile(targets),
      'mass_nominal_kg':1.8048682,'torque_nominal_leg_cap_Nm':.7,
      'gravity_m_s2':9.81,'simulation_dt_s':.002,'servo_control_law':'kp7 kd0.32 bounded torque',
      'original_Freenove_PWM_limits_calibrated':False,'runtime_browser_mujoco':False,
      'data_files_count':verified_count,'variant_counts':{k:len(v) for k,v in results.items()},
      'physics_computed_not_animation':True,'free_root':True,'leg_contacts_real_MuJoCo':True,'cases':cases,
      'interpretation':'Model locomotion staying upright for finite duration does not establish contact consistency, PWM feasibility or command tracking.'}
    (HERE/'SUMMARY_V12.json').write_text(json.dumps(verified,ensure_ascii=False,indent=2)+'\n')
    n=cases['nominal_full']; f=cases['friction_035_full'];t=cases['torque_035_full'];up=cases['upper_mass_full'];
    assert n['stand_yaw_plus15']['min_real_contacts']==6
    assert 14.5<n['stand_yaw_plus15']['actual_yaw_at_active_end_deg']<15.5
    assert n['stand_yaw_plus15']['max_torque_Nm']<.2
    assert n['tripod_forward']['min_real_contacts']<3
    assert n['tripod_turn']['time_with_no_contacts_s']>0
    assert n['wave_forward']['min_real_contacts']<3
    assert n['tripod_turn']['actual_yaw_at_active_end_deg']>180
    assert n['tripod_turn']['command_odometry_yaw_deg']<65
    assert t['tripod_forward']['saturation_fraction']>.6
    assert f['tripod_forward']['loaded_contact_fraction_speed_gt_5mm_s']>.4
    assert up['tripod_forward']['saturation_fraction']>.08
    def fmt(row,name):
        r=row[name];return f"{r['actual_net_xy_m']*100:.1f} см | {r['actual_yaw_unwrapped_deg']:+.1f}° | {r['min_real_contacts']} | {r['max_torque_Nm']:.2f} | {100*r['saturation_fraction']:.1f}% | {r['max_loaded_contact_slip_speed_mm_s']:.1f} мм/с"
    head='''# CAD30 / Freenove V11 — физическая проверка MuJoCo (V12)

**6 ноября 2026. Статус: физическая проверка в модели выполнена; пригодность реального робота НЕ подтверждена.**

## Протокол, не анимация

Использован исходный приватный Freenove FNK0052 CAD30 MJCF (SHA-256 `fe9d7c29…`) и заводские V7 таблицы, как они действительно подключены в V11. Исходные целевые углы для каждой ноги вычислены **оригинальным JavaScript `PlantedCombinedPilot`**, а не переписаны вручную на Python. MuJoCo 3.14.0 интегрировал свободный корпус через `mj_step` с гравитацией 9,81 м/с², шестью контактными tibia-мешами, столкновениями и 20 моторными каналами. Никакой принудительной установки корневого положения, root yaw, фиксации стоп или использования экранного `GroundLock` для физического движения.

**Физические допущения модели:** номинальная масса 1,805 кг; PD-привод `τ=clip(7(q_command−q)−0.32q̇,±0.7 Н·м)` для ног, голова ±0,2 Н·м. Частота физики 500 Гц, целевые углы JS каждые 10 мс, начальная стойка удерживалась под гравитацией ещё 1 с. Исходный коэффициент трения на тибиях 1,0; в тесте ослабленного сцепления он установлен 0,35 **и на стопах, и на полу**, с учётом приоритета MuJoCo friction. Контакт считается действительным по `mj_contactForce`, скорость относительного скольжения вычислена якобианом в контактной точке только при нормальной силе ≥0,5 Н. Поддержка по COM — отдельная геометрическая оценка, не подмена динамики.

Всего **53 полноценных физических прогона**: 29 номинальных, 10 с верхней массой 2,08 кг, 7 со сниженным трением и 7 с моментом ног до 0,35 Н·м. Горизонты 1–5,5 с после стабилизации стойки; ни один прогон не завершился падением по порогам высоты 75 мм и наклона 45°. Это не означает, что все движения прошли необходимые критерии качества.

## Результат для неподвижного корпуса

Изолированные OEM команды: разворот ±15°, крен/тангаж ±15°, высота ±20 мм и смещения XY ±40 мм — в номинальной модели в целом удаётся удержать без падения и насыщения. Для разворота +15° фактический пик ~15,11°, все 6 ног в контакте, момент ≤0,141 Н·м, контактное скольжение ≤0,244 мм/с. Разворот −15° симметричен. Высота ±20 мм корректно реализована. **Исключение:** сочетание крена+тангажа по 10° не падает, но временно имеет только **две** реальные опоры, COM проекция выходит за их многоугольник, скорость скольжения на нагруженной стопе до 31,7 мм/с. При трении 0,35 даже изолированный разворот теряет часть из шести контактов и показывает скольжение до 10,3 мм/с.

## Заводская ходьба (реальная интеграция, номинальная модель)

| Сценарий | Расчётная команда | Результат физики | Минимум опор | Момент (Н·м) | Насыщение шагов | Макс. скорость нагруженного контакта |
|---|---:|---:|---:|---:|---:|---:|
'''
    rows=[('tripod_forward','Tripod вперёд','dist'),('tripod_turn','Tripod поворот','yaw'),('tripod_forward_turn','Tripod вперёд+поворот','both'),('wave_forward','Wave вперёд','dist'),('wave_turn','Wave поворот','yaw'),('tripod_forward_roll','Tripod вперёд+крен','dist'),('wave_forward_roll','Wave вперёд+крен','dist')]
    for key,name,mode in rows:
        r=n[key]
        requested=(f"{r['visual_odom_net_xy_m']*100:.1f} см" if mode=='dist' else
                   f"{r['command_odometry_yaw_deg']:.1f}°" if mode=='yaw' else
                   f"{r['visual_odom_net_xy_m']*100:.1f} см + {r['command_odometry_yaw_deg']:.1f}°")
        obtained=(f"{r['actual_net_xy_m']*100:.1f} см" if mode=='dist' else
                  f"{r['actual_yaw_unwrapped_deg']:.1f}°" if mode=='yaw' else
                  f"{r['actual_net_xy_m']*100:.1f} см + {r['actual_yaw_unwrapped_deg']:.1f}°")
        head+=f"| {name} | {requested} | {obtained} | {r['min_real_contacts']} | {r['max_torque_Nm']:.2f} | {r['saturation_fraction']*100:.1f}% | {r['max_loaded_contact_slip_speed_mm_s']:.1f} мм/с |\n"
    foot='''
Все приведённые значения «расчётная команда» — исходная **виртуальная одометрия** из FactoryPilot, а не физически заданный свободный root. Фактический угол после нескольких циклов развёрнут по ±180°, чтобы не потерять полные обороты. **Tripod поворот: +217,5° фактически против +62,5° в одометрии**. Подбор движений тела посредством экранного `GroundLock` не влияет на поведение физического робота. Максимальный тангенциальный контактный показатель в таблице — скорость, а не накопленное расстояние проскальзывания.

При номинальной походке Tripod обнаружены интервалы **менее трёх реальных опор**, вплоть до отсутствия нагрузки на стопах в отдельные отсчёты поворота; некоторые суставные команды упираются в ограничение 0,7 Н·м. Wave не падает и вперёд продвигается почти на заданное расстояние, но также кратковременно снижает число опор до двух и имеет контактное движение. **Ни одна из проверенных походок не может быть помечена как физически подтверждённая для реального устройства.**

## Проверки чувствительности

| Изменение | Наблюдение |
|---|---|
'''
    foot+=f"| Номинальная масса 1,80 → 2,08 кг | Насыщение Tripod вперёд {n['tripod_forward']['saturation_fraction']*100:.1f}% → {up['tripod_forward']['saturation_fraction']*100:.1f}% |\n"
    foot+=f"| Коэффициент трения 1 → 0,35 | В Tripod вперёд доля нагруженных контактов со скоростью >5 мм/с {n['tripod_forward']['loaded_contact_fraction_speed_gt_5mm_s']*100:.1f}% → {f['tripod_forward']['loaded_contact_fraction_speed_gt_5mm_s']*100:.1f}%; минимальное число опор {n['tripod_forward']['min_real_contacts']} → {f['tripod_forward']['min_real_contacts']} |\n"
    foot+=f"| Лимит момента 0,7 → 0,35 Н·м | Tripod вперёд: насыщение {n['tripod_forward']['saturation_fraction']*100:.1f}% → {t['tripod_forward']['saturation_fraction']*100:.1f}% |\n"
    foot+=f"| Уменьшение темпа Tripod в 2 раза | Насыщение {n['tripod_forward']['saturation_fraction']*100:.1f}% → {n['tripod_forward_slow']['saturation_fraction']*100:.1f}%, но минимум опор остаётся {n['tripod_forward_slow']['min_real_contacts']}, а поворот за 4,5 с {n['tripod_turn_slow']['actual_yaw_unwrapped_deg']:.1f}° против {n['tripod_turn_slow']['command_odometry_yaw_deg']:.1f}° расчетных |\n"
    foot+='''
## Что означает «прошло»

- **Выполнено в динамической модели:** изолированный разворот/наклон/смещение корпуса из штатной стойки при указанных параметрах; реальная гравитация, контактные силы и PD-моменты учитывались. Статический разворот 15° выдерживает 6 опор на номинальном сцеплении.
- **Не выполнены критерии надежной походки:** сохранение ≥3 нагруженных опор в каждой фазе, малая скорость проскальзывания нагруженных стоп, адекватная виртуальная одометрия и запас по моменту (особенно Tripod). Временное отсутствие ≥3 контактов не обязательно означает мгновенное падение благодаря инерции, однако это физическая проблема для робота с открытым контуром.
- **Не тестировано на железе:** реальные калибровки нулевых углов, точное расположение опор, реальные момент-скорость/PWM кривые, температура моторов, люфты, ударные нагрузки, батарея, контроллер и задержки Wi-Fi. Физическая симуляция не является сертификатом безопасности.

## Воспроизводимость

Распаковать приватный CAD30 с оригинальными 30 STL и MJCF **локально**; не класть его в публичный GitHub. Исходные скрипты, контрольные суммы, точные 29 профилей суставных команд и полные записи 53 прогонов находятся в ZIP-артефакте на Drive. Использовать MuJoCo 3.14.0, Python 3.13 и Node 22:

```bash
node research/physics_v12/generate_targets_v11.mjs research/physics_v12/data/targets_v11.json
PYTHONPATH=. python research/physics_v12/run_physics_v11.py --out research/physics_v12/results/nominal_full
PYTHONPATH=. python research/physics_v12/run_physics_v11.py --scene source/mjcf/scene_local30_upper.xml --out research/physics_v12/results/upper_mass_full --cases stand_yaw_plus15 stand_roll15 stand_roll_pitch10 tripod_forward tripod_turn tripod_forward_turn tripod_forward_roll wave_forward wave_turn wave_forward_roll
PYTHONPATH=. python research/physics_v12/run_physics_v11.py --friction-scale .35 --out research/physics_v12/results/friction_035_full --cases stand_yaw_plus15 stand_roll15 tripod_forward tripod_turn tripod_forward_turn wave_forward wave_turn
PYTHONPATH=. python research/physics_v12/run_physics_v11.py --torque-cap .35 --out research/physics_v12/results/torque_035_full --cases stand_yaw_plus15 stand_roll15 tripod_forward tripod_turn tripod_forward_turn wave_forward wave_turn
python research/physics_v12/summarize_evidence.py
python -m unittest research/physics_v12/test_physics_v12.py -v
```

Не сливать экспериментальный PR в `main`; не включать этот результат в моторные команды и не отмечать походку как безопасную. В плеере физического интегратора по-прежнему нет.
'''
    (HERE/'REPORT_V12.md').write_text(head+foot)
    print('VERIFIED',verified_count,'cases; generated',HERE/'SUMMARY_V12.json','and report bytes',len((HERE/'REPORT_V12.md').read_bytes()))

if __name__=='__main__':main()
