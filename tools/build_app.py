"""Build the static preview site into app/dist.

data.js embeds experiments/index.json, the dataset catalog, the dataset CSV files and the markdown
documents, so the site also works when opened from disk. Figures are copied next to the page with
the same relative paths the manifest uses.
"""

from __future__ import annotations

import json
import shutil

import registry as R

SRC = R.ROOT / "app" / "src"
DIST = R.ROOT / "app" / "dist"


def main():
    if DIST.exists():
        shutil.rmtree(DIST)
    DIST.mkdir(parents=True)
    for f in ("index.html", "style.css", "app.js"):
        shutil.copy(SRC / f, DIST / f)
    index = json.loads((R.ROOT / "experiments" / "index.json").read_text())
    catalog = json.loads((R.ROOT / "datasets" / "catalog" / "catalog.json").read_text())
    csv = {d["file"]: (R.ROOT / d["file"]).read_text() for d in catalog["datasets"]}
    docs = {p.name: p.read_text() for p in sorted((R.ROOT / "docs").glob("*.md"))}
    for p in sorted((R.ROOT / "research").rglob("*.md")):
        docs["research/" + str(p.relative_to(R.ROOT / "research")).replace("\\", "/")] = p.read_text()
    payload = dict(index=index, catalog=catalog, csv=csv, docs=docs)
    (DIST / "data.js").write_text("window.OC_DATA = " + json.dumps(payload, ensure_ascii=False) + ";\n")
    n = 0
    for e in index["experiments"]:
        for fig in e["figures"]:
            dst = DIST / fig
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(R.ROOT / fig, dst)
            n += 1
    print(f"app built: {DIST.relative_to(R.ROOT)}, {n} figures, {len(docs)} documents, "
          f"{sum(p.stat().st_size for p in DIST.rglob('*') if p.is_file()) // 1024} KiB")


if __name__ == "__main__":
    main()
