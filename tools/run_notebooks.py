"""Execute notebooks from a clean kernel, then check the results they must produce.

usage: python tools/run_notebooks.py [spec_name ...]
Exit code is non-zero if a notebook raises or a declared result file is missing or invalid.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import nbformat  # noqa: E402
import nbgen  # noqa: E402
from nbclient import NotebookClient  # noqa: E402
from specs import ORDER  # noqa: E402

ROOT = nbgen.ROOT


def run(name: str) -> dict:
    path = nbgen.write_notebook(name)
    _, meta = nbgen.build_notebook(name)
    nb = nbformat.read(path, as_version=4)
    os.environ["CONTROL_LAB_ROOT"] = str(ROOT)
    t0 = time.time()
    client = NotebookClient(nb, timeout=900, kernel_name="python3",
                            resources={"metadata": {"path": str(path.parent)}})
    client.execute()
    nbformat.write(nb, path)
    elapsed = time.time() - t0
    missing, bad = [], []
    for exp in meta["experiments"]:
        f = ROOT / "experiments" / "results" / f"{exp}.json"
        if not f.exists():
            missing.append(exp)
            continue
        data = json.loads(f.read_text())
        if data["result_status"] != "completed":
            bad.append(f"{exp}: result_status={data['result_status']}")
        failed = [c["name"] for c in data["checks"] if not c["passed"]]
        if failed:
            bad.append(f"{exp}: failed checks {failed}")
    return {"name": name, "seconds": round(elapsed, 1), "missing": missing, "problems": bad}


def main(names=None) -> int:
    code = 0
    for name in names or ORDER:
        try:
            r = run(name)
        except ModuleNotFoundError as exc:
            if names is None and exc.name and exc.name.endswith(name):
                print(f"skip (spec not written yet): {name}")
                continue
            raise
        status = "OK" if not (r["missing"] or r["problems"]) else "PROBLEM"
        print(f"{status:8s} {name} {r['seconds']}s missing={r['missing']} "
              f"problems={r['problems']}")
        if status != "OK":
            code = 1
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or None))
