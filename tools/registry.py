"""Shared loaders for the generator scripts. Reads specs, notebooks, result files and refs."""

from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

import refs  # noqa: E402
from nbgen import SECTION_TITLES  # noqa: E402
from specs import ORDER  # noqa: E402

RESULTS = ROOT / "experiments" / "results"


def load_metas() -> list[dict]:
    out = []
    for name in ORDER:
        mod = importlib.import_module(f"specs.{name}")
        m = dict(mod.META)
        m["spec"] = name
        m["number"] = m["id"].split("_")[0]
        m["notebook"] = f"notebooks/{m['folder']}/{m['id']}.ipynb"
        out.append(m)
    return out


def load_result(exp_id: str) -> dict | None:
    p = RESULTS / f"{exp_id}.json"
    return json.loads(p.read_text()) if p.exists() else None


def notebook_headings(path: Path) -> list[str]:
    nb = json.loads(path.read_text())
    heads = []
    for c in nb["cells"]:
        if c["cell_type"] == "markdown":
            first = "".join(c["source"]).split("\n")[0]
            if first.startswith("## "):
                heads.append(first[3:])
    return heads


def expected_headings() -> list[str]:
    return [f"{n}. {t}" for n, t in SECTION_TITLES.items()]


def content_status(meta: dict) -> str:
    p = ROOT / meta["notebook"]
    if not p.exists():
        return "Missing"
    return "Complete (15/15 sections)" if notebook_headings(p) == expected_headings() else "Incomplete"


def ref_summary(meta: dict) -> dict:
    keys = list(meta.get("refs", []))
    stds = list(meta.get("standards", []))
    items = [dict(key=k, kind="reference", status=refs.REFS[k]["verification"]["status"]) for k in keys]
    items += [dict(key=k, kind="standard", status=refs.STANDARDS[k]["verification"]["status"]) for k in stds]
    n_ver = sum(1 for i in items if i["status"] == "verified")
    return dict(items=items, verified=n_ver, partial=len(items) - n_ver, total=len(items))


def checks_summary(result: dict | None) -> dict:
    if not result:
        return dict(total=0, passed=0)
    c = result["checks"]
    return dict(total=len(c), passed=sum(1 for x in c if x["passed"]))


def notebook_ids_for_topic(row) -> list[str]:
    return row["notebooks"]


def topic_counts(rows) -> dict:
    out: dict[str, int] = {}
    for r in rows:
        out[r["status"]] = out.get(r["status"], 0) + 1
    return out


def write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False, sort_keys=False) + "\n")
