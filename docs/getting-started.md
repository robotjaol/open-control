# Getting started

## Install
Python 3.10 or newer (built and tested here on 3.13).

```bash
git clone https://github.com/robotjaol/open-control.git
cd open-control
pip install -e ".[development]"
```

## Run the tests
```bash
make test             # unit, integration and simulation tests
make test-notebooks   # structural checks on notebooks and results (no re-execution)
make lint
```

## Reproduce a notebook
```bash
python tools/build_notebooks.py          # specs -> .ipynb
python tools/run_notebooks.py s11_conveyor_speed   # spec name; no argument runs all
```
`run_notebooks.py` executes the notebook from a clean kernel (the result file is validated against the
schema when the notebook saves it) and exits with an error if a result file is missing, a check fails
or the result is not `completed`. Total run time on the build machine is stated in `CHANGELOG.md`.

## Regenerate everything derived
```bash
make research catalog validate
python tools/build_experiment_index.py
python tools/build_docs.py
make app              # preview website into app/dist
```
Generated files carry a header comment. Edit the source (specs, `tools/refs.py`, `tools/taxonomy.py`),
never the output.

## Where things are
| Path | Content |
|---|---|
| `src/control_lab/` | Models, controllers, simulation loop, identification, experiment tracking |
| `notebooks/` | 12 research notebooks, 15 sections each |
| `tools/specs/` | Notebook sources (one spec per notebook) |
| `experiments/` | Result JSON, figures, metadata, configurations, schema, `index.json` |
| `research/` | Source, standards and algorithm registers; comparisons; gaps |
| `datasets/` | Catalog, schema, one synthetic example |
| `docs/` | Roadmap, methodology, status, taxonomy, labs |
| `app/` | Preview website (static, reads `experiments/index.json`) |

## Using the library
```python
from control_lab.models import BeltConveyor
from control_lab.controllers import PID
from control_lab.simulation import run_closed_loop
```
See the notebooks for complete, executed examples.
