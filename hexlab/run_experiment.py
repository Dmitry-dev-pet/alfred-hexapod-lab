"""Run a controlled kinematic comparison; execute from project directory."""
import json
import pathlib
from hexlab.simulation import simulate


def main():
    cases = [
        simulate('tripod'),
        simulate('decentralized'),
        simulate('tripod', fail_leg_at=6),
        simulate('decentralized', fail_leg_at=6),
    ]
    out = pathlib.Path('results')
    out.mkdir(exist_ok=True)
    (out/'pilot_results.json').write_text(json.dumps(cases, ensure_ascii=False, indent=2))
    for case in cases:
        print('%-14s broken=%-5s unstable=%5.1f%% min_margin=%s mm steps=%d' % (
            case['strategy'],case['failed_leg'],100*case['fraction_geometrically_unstable'],
            case['min_margin_mm'],case['foot_swing_count']))


if __name__ == '__main__':
    main()