"""Final bounded target-margin check, retaining unchanged physical limits."""
from pathlib import Path
import argparse
import hashlib
import json
import tune_recovery as sim

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workspace', type=Path, required=True)
    a = ap.parse_args()
    sim.initialize(a.workspace)
    path = HERE / 'traces/summary.json'
    summary = json.loads(path.read_text())
    assert 'final_margin_check' not in summary
    base = summary['selected']['params']
    extra = []
    for height in (.01, .02):
        for margin in (.28, .35):
            params = {**base, 'height_m': height, 'motor_target_margin_rad': margin}
            extra.append(sim.save(params, f'final-h{round(height*100)}-margin{round(margin*100)}', HERE / 'traces'))
    accepted = [e for e in extra if e['status'] == 'PASS']
    accepted.sort(key=lambda e: (-e['params']['height_m'], e['params']['motor_target_margin_rad']))
    selected = accepted[0] if accepted else None
    if selected:
        for seed in (20261109, 20261110, 20261111):
            extra.append(sim.save({**selected['params'], 'seed': seed}, f'final-perturb-{seed}', HERE / 'traces'))
        trace, _ = sim.run(selected['params'])
        raw = (json.dumps(trace, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()
        assert hashlib.sha256(raw).hexdigest() == selected['trace_sha256']
    summary['episodes'].extend(extra)
    summary['final_margin_check'] = {'tested_margins_rad': [.28, .35], 'selected': selected,
                                   'byte_identical_repeat': bool(selected)}
    path.write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
