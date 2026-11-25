# Contributing

Thank you for helping. The standard here is that every claim can be checked.

## Before you start
- Open an issue describing the topic, the plant or algorithm, and the reference you intend to rely on.
- Read [methodology](docs/methodology.md) and [simulation lab](docs/simulation-lab.md).

## Rules for a contribution
1. **Results come from code.** No number in a notebook or document is typed from memory or from a paper.
2. **Every notebook has 15 sections** (see `tools/nbgen.py`) and a limitations section written after the results were inspected.
3. **Checks compare against something independent** (closed form, conservation law, exact discretisation, second solver). Do not loosen a tolerance to make a check pass; find out whether the test, the library or the reference is wrong. Disclose any threshold chosen after seeing data.
4. **References are verified before they are added** to `tools/refs.py`, with the evidence URL, the date and what was not confirmed. Use status `partial` when something could not be confirmed. Never add a reference from memory.
5. **Label data honestly**: `synthetic_illustrative`, `simulated`, `public_dataset` (with licence and source), `measured_experiment` (with instruments and calibration).
6. **Hardware validation** needs the evidence listed in [hardware lab](docs/hardware-lab.md). Do not change a status to validated without it.
7. Do not add empty folders or placeholder pages.

## Workflow
```bash
pip install -e ".[development]"
ruff check src tests scripts tools
make test
python tools/run_notebooks.py sNN_name
python tools/build_experiment_index.py && python tools/build_research.py \
  && python tools/build_catalog.py && python tools/build_docs.py && python tools/build_readme.py
make validate
```
Generated files carry a header; edit their sources instead.

## Pull requests
Describe what was verified and how, and what was not. State which status dimensions change (content, software, research, simulation, hardware).
