"""Validate every experiments/results/*.json against the result schema and the project's rules.

Exit code 1 if any file is invalid, partial, failed, claims hardware, or has a failed check.
"""

import json
import sys
from pathlib import Path

import jsonschema

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    schema = json.loads((ROOT / "experiments/schema/experiment_result.schema.json").read_text())
    files = sorted((ROOT / "experiments/results").glob("*.json"))
    bad = 0
    for f in files:
        d = json.loads(f.read_text())
        errs = [e.message for e in jsonschema.Draft202012Validator(schema).iter_errors(d)]
        if d.get("result_status") != "completed":
            errs.append(f"result_status is {d.get('result_status')}")
        if d.get("experiment_type") != "software_simulation":
            errs.append(f"unexpected experiment_type {d.get('experiment_type')} (no hardware run exists)")
        if d.get("hardware_configuration"):
            errs.append("hardware_configuration is set but no hardware experiment exists")
        failed = [c["name"] for c in d["checks"] if not c["passed"]]
        if failed:
            errs.append(f"failed checks: {failed}")
        for fig in d["figure_references"]:
            if not (ROOT / fig).exists():
                errs.append(f"missing figure {fig}")
        print(("FAIL" if errs else "ok  "), f.name, f"{len(d['checks'])} checks")
        for e in errs:
            print("    ", e)
        bad += bool(errs)
    print(f"{len(files)} result files, {bad} invalid")
    return 1 if bad or not files else 0


if __name__ == "__main__":
    sys.exit(main())
