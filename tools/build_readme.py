"""Generate README.md. Counts come from experiments/index.json; nothing is typed by hand."""

from __future__ import annotations

import json

import registry as R
import taxonomy


def main():
    idx = json.loads((R.ROOT / "experiments" / "index.json").read_text())
    s = idx["summary"]
    ts = s["topic_status"]
    metas = R.load_metas()
    rows = []
    for m in metas:
        e = next(x for x in idx["experiments"] if x["experiment_id"] == m["experiments"][0])
        rows.append(f"| {m['number']} | [{m['title']}]({m['notebook']}) | {taxonomy.difficulty(m['level'])} | "
                    f"{e['status']['simulation'].split(' (')[0]} |")
    nb_table = "| No. | Notebook | Difficulty | Simulation checks |\n|---|---|---|---|\n" + "\n".join(rows)
    projects = idx["projects"]
    pc = {k: sum(1 for p in projects if p["status"] == k) for k in ("In Progress", "Planned", "Blocked")}
    text = f"""# open-control

A research-based laboratory for **industrial control engineering**: a tested Python library
(`control_lab`), {s['notebooks']} executed research notebooks, structured experiment results, a register of
sources with their verification status, and a static preview website that reads the results.

Modeled on the structure of [open-python](https://github.com/robotjaol/open-python), not copied from it.

## Honest status

| Dimension | Status |
|---|---|
| Content | {s['notebooks']} notebooks, each with all 15 required sections. Coverage of the full topic list is small: see below. |
| Software | Library unit tests, integration, simulation and notebook-structure tests passed on the build machine. **GitHub Actions workflows exist but have not been run**; the repository was not pushed from the build environment. |
| Simulation | {s['checks_passed']} of {s['checks_total']} automated checks passed across {s['experiments']} experiments. Every check compares against an analytic or independent reference. |
| Research | {s['references']} references and {s['standards']} standards registered, verified by web search of bibliographic records (some only partly). No full text was read. No literature from 2022 onward is registered. |
| Hardware | **Not validated. No physical experiment, hardware-in-the-loop run or firmware exists.** |

All plant parameters are illustrative. A passing check means the code reproduces a closed form,
a conservation law or an independent solver. It does not mean a model matches a real machine.

Topic coverage: {s['topics']} topics defined, {ts.get('Simulation Validated', 0)} Simulation Validated,
{ts.get('In Progress', 0)} In Progress, {ts.get('Planned', 0)} Planned, 0 Hardware Validated
([taxonomy](docs/topic-taxonomy.md)). Projects: {pc['In Progress']} In Progress, {pc['Planned']} Planned,
{pc['Blocked']} Blocked ([catalog](docs/projects.md)). Per-notebook detail: [validation status](docs/validation-status.md).

## Notebooks

{nb_table}

Each notebook follows the same 15 sections, from learning objectives to exercises and further research,
and states its limitations. Notebook sources are in `tools/specs/`; the `.ipynb` files are generated and executed.

## Quick start

```bash
git clone https://github.com/robotjaol/open-control.git
cd open-control
pip install -e ".[development]"
make test                      # unit, integration, simulation tests
python tools/run_notebooks.py  # re-execute all notebooks from a clean kernel
make app                       # build the preview site into app/dist
```
See [getting started](docs/getting-started.md).

## What is where

| Path | Content |
|---|---|
| `src/control_lab/` | Models (LTI, tank, belt conveyor), motors (DC, BLDC, PMSM), controllers (PID, tuning, cascade, fuzzy, FSM, PLC blocks, FOC, MPC), simulation loop, experiment tracking |
| `notebooks/` | Research notebooks |
| `experiments/` | Result JSON (schema-validated), figures, metadata, configurations, `index.json` |
| `research/` | Source, algorithm and standards registers, algorithm comparisons, research gaps |
| `datasets/` | Catalog, schema, one synthetic example. **No measured dataset is included.** |
| `docs/` | [Learning roadmap](docs/learning-roadmap.md), [methodology](docs/methodology.md), [experiment schema](docs/experiment-schema.md), [simulation lab](docs/simulation-lab.md), [hardware lab](docs/hardware-lab.md) |
| `app/` | Preview website: Overview, Learning Paths, Algorithm Library, Motor Lab, PLC, Process, Research Library, Dataset Catalog, Experiment Gallery, Projects, Docs |

## Integrity rules
- Results are produced by code in this repository and stored with their configuration.
- Synthetic data is labelled synthetic. Hardware validation is never implied.
- A reference is listed as verified only if its bibliographic record was found; partial and unverified items are marked.
- A failing check is investigated, not loosened. Thresholds chosen after seeing data are disclosed in the notebook.
- Incomplete is reported as incomplete.

## Known limitations
- Single-input examples; no multivariable, estimation, adaptive or robust control yet.
- Fuzzy and MPC results are specific to one plant and one rule base or horizon set each.
- Timing measurements are specific to the build machine.
- Standards were checked at catalogue level only. IEC 61131-3:2013 has a newer edition (2025) that was not examined.

## Contributing, security, license
See [CONTRIBUTING](CONTRIBUTING.md), [SECURITY](SECURITY.md) and [CODE OF CONDUCT](CODE_OF_CONDUCT.md). MIT license. Cite with `CITATION.cff`.
"""
    (R.ROOT / "README.md").write_text(text)
    print("README written")


if __name__ == "__main__":
    main()
