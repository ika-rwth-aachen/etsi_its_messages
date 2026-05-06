#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys


def repo_root_from_script(script: Path) -> Path:
    return script.resolve().parents[3]


IVIM_CONVERTER_ALIASES = {
    "Temperature": "Temperature2",
    "TrailerDetails": "TrailerCharacteristicsFixValuesList",
    "TrailerAxles": "TrailerCharacteristicsRangesList",
}


def collect_msg_names(msg_dir: Path) -> set[str]:
    return {p.stem for p in msg_dir.glob("*.msg")}


def collect_converter_names(conv_dir: Path) -> set[str]:
    names: set[str] = set()
    for p in conv_dir.glob("convert*.h"):
        stem = p.stem
        if stem.startswith("convert") and len(stem) > len("convert"):
            names.add(stem[len("convert"):])
    return names


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit IVIM conversion header coverage against IVIM ROS messages.")
    parser.add_argument("--repo-root", type=Path, default=None, help="Repository root. Auto-detected from script location when omitted.")
    parser.add_argument("--report", type=Path, default=None, help="Optional report file to write.")
    args = parser.parse_args()

    script = Path(__file__)
    repo_root = args.repo_root.resolve() if args.repo_root else repo_root_from_script(script)

    msg_dir = repo_root / "etsi_its_msgs/etsi_its_ivim_ts_msgs/msg"
    conv_dir = repo_root / "etsi_its_conversion/etsi_its_ivim_ts_conversion/include/etsi_its_ivim_ts_conversion"

    if not msg_dir.is_dir():
        print(f"Missing IVIM msg directory: {msg_dir}", file=sys.stderr)
        return 2
    if not conv_dir.is_dir():
        print(f"Missing IVIM conversion include directory: {conv_dir}", file=sys.stderr)
        return 2

    msgs = collect_msg_names(msg_dir)
    converters = collect_converter_names(conv_dir)

    missing = sorted(msgs - converters)
    extra = sorted(converters - msgs)

    important_roots = [
        "IVIM", "IviStructure", "IviManagementContainer", "IviContainers", "IviContainer",
        "GeographicLocationContainer", "GeneralIviContainer", "RoadConfigurationContainer",
        "TextContainer", "LayoutContainer", "AutomatedVehicleContainer", "MapLocationContainer",
        "RoadSurfaceContainer",
    ]
    missing_roots = [name for name in important_roots if name in msgs and name not in converters]

    lines = []
    lines.append("IVIM conversion coverage audit")
    lines.append(f"Repository: {repo_root}")
    lines.append(f"Message types: {len(msgs)}")
    lines.append(f"Conversion headers: {len(converters)}")
    lines.append(f"Missing converters: {len(missing)}")
    lines.append(f"Extra converters: {len(extra)}")
    if missing_roots:
        lines.append("Missing critical root/container converters: " + ", ".join(missing_roots))
    if missing:
        lines.append("Missing converter headers:")
        lines.extend(f"  - convert{name}.h" for name in missing)
    if extra:
        lines.append("Extra converter headers without matching .msg:")
        lines.extend(f"  - convert{name}.h" for name in extra)
    if not missing and not extra:
        lines.append("Coverage OK: every IVIM ROS message has a matching convert*.h header.")

    output = '\n'.join(lines) + '\n'
    sys.stdout.write(output)
    if args.report:
        args.report.write_text(output)

    return 1 if (missing or extra) else 0


if __name__ == "__main__":
    raise SystemExit(main())
