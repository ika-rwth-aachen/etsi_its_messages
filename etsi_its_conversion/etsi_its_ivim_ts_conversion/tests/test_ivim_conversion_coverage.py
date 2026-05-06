#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys


def main() -> int:
    repo_root = Path(__file__).resolve().parents[3]
    audit = repo_root / 'etsi_its_conversion' / 'etsi_its_ivim_ts_conversion' / 'scripts' / 'audit_conversion_coverage.py'
    report = repo_root / 'etsi_its_conversion' / 'etsi_its_ivim_ts_conversion' / 'IVIM_CONVERSION_COVERAGE.txt'

    proc = subprocess.run(
        [sys.executable, str(audit), '--repo-root', str(repo_root), '--report', str(report)],
        text=True,
        capture_output=True,
    )
    sys.stdout.write(proc.stdout)
    sys.stderr.write(proc.stderr)
    if proc.returncode != 0:
        return proc.returncode
    txt = report.read_text(encoding='utf-8')
    if 'Missing converters: 0' not in txt or 'Extra converters: 0' not in txt:
        print('Coverage report is not clean', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
