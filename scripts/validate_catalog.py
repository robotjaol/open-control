"""Validate datasets/catalog/catalog.json: schema, file existence, row count and checksum."""

import hashlib
import json
import sys
from pathlib import Path

import jsonschema
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    schema = json.loads((ROOT / "datasets/schemas/dataset_metadata.schema.json").read_text())
    cat = json.loads((ROOT / "datasets/catalog/catalog.json").read_text())
    bad = 0
    for e in cat["datasets"]:
        errs = [x.message for x in jsonschema.Draft202012Validator(schema).iter_errors(e)]
        p = ROOT / e["file"]
        if not p.exists():
            errs.append("file missing")
        else:
            if hashlib.sha256(p.read_bytes()).hexdigest() != e["sha256"]:
                errs.append("sha256 mismatch")
            df = pd.read_csv(p)
            if len(df) != e["n_rows"]:
                errs.append(f"n_rows {e['n_rows']} != {len(df)}")
            if list(df.columns) != [c["name"] for c in e["columns"]]:
                errs.append("columns differ from the metadata")
        if e["kind"] in ("synthetic_illustrative", "simulated") and "synthetic" not in (e["name"] + e["description"]).lower() \
                and "generated" not in e["description"].lower():
            errs.append("synthetic data is not labelled as synthetic/generated in name or description")
        print(("FAIL" if errs else "ok  "), e["id"])
        for m in errs:
            print("    ", m)
        bad += bool(errs)
    print(f"{len(cat['datasets'])} dataset(s), {bad} invalid")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
