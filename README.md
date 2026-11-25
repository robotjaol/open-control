# open-control

A research-based laboratory for **industrial control engineering**: a tested Python library
(`control_lab`), 12 executed research notebooks, structured experiment results, a register of
sources with their verification status, and a static preview website that reads the results.

Modeled on the structure of [open-python](https://github.com/robotjaol/open-python), not copied from it.

## Honest status

| Dimension | Status |
|---|---|
| Content | 12 notebooks, each with all 15 required sections. Coverage of the full topic list is small: see below. |
| Software | Library unit tests, integration, simulation and notebook-structure tests passed on the build machine. **GitHub Actions workflows exist but have not been run**; the repository was not pushed from the build environment. |
| Simulation | 179 of 179 automated checks passed across 12 experiments. Every check compares against an analytic or independent reference. |
| Research | 19 references and 2 standards registered, verified by web search of bibliographic records (some only partly). No full text was read. No literature from 2022 onward is registered. |
| Hardware | **Not validated. No physical experiment, hardware-in-the-loop run or firmware exists.** |

All plant parameters are illustrative. A passing check means the code reproduces a closed form,
a conservation law or an independent solver. It does not mean a model matches a real machine.

Topic coverage: 283 topics defined, 50 Simulation Validated,
33 In Progress, 200 Planned, 0 Hardware Validated
([taxonomy](docs/topic-taxonomy.md)). Projects: 8 In Progress, 6 Planned,
1 Blocked ([catalog](docs/projects.md)). Per-notebook detail: [validation status](docs/validation-status.md).

## Notebooks

| No. | Notebook | Difficulty | Simulation checks |
|---|---|---|---|
| 01 | [Control Fundamentals: Feedback, Error and Stability](notebooks/01_fundamentals/01_control_fundamentals.ipynb) | Beginner | 16/16 checks passed |
| 02 | [Transfer Function Modeling and FOPDT Identification](notebooks/01_fundamentals/02_transfer_function_modeling.ipynb) | Beginner | 6/6 checks passed |
| 03 | [PID Control: Structure, Implementation and Anti-Windup](notebooks/02_classical/03_pid.ipynb) | Beginner | 10/10 checks passed |
| 04 | [PID Tuning Rules: Comparison on FOPDT Plants](notebooks/02_classical/04_pid_tuning.ipynb) | Intermediate | 9/9 checks passed |
| 05 | [Finite State Machines for Industrial Sequencing](notebooks/03_discrete/05_fsm.ipynb) | Intermediate | 17/17 checks passed |
| 06 | [DC Motor Speed Control: Single Loop versus Cascade with Current Limit](notebooks/04_motor_control/06_dc_motor_speed.ipynb) | Intermediate | 12/12 checks passed |
| 07 | [PLC Sequence Control: Scan Cycle, Timers, Counters and a Batching Station](notebooks/03_discrete/07_plc_sequence.ipynb) | Intermediate | 23/23 checks passed |
| 08 | [Fuzzy PI Control of a Nonlinear Tank: A Budget-Matched Comparison with PI](notebooks/05_intelligent/08_fuzzy.ipynb) | Intermediate | 17/17 checks passed |
| 09 | [BLDC Six-Step Commutation: Alignment, Torque Ripple and Speed Control](notebooks/04_motor_control/09_bldc_six_step.ipynb) | Intermediate | 13/13 checks passed |
| 10 | [PMSM Field-Oriented Control: Transforms, Current Loops, Decoupling and the Voltage Limit](notebooks/04_motor_control/10_pmsm_foc.ipynb) | Advanced | 20/20 checks passed |
| 11 | [Belt Conveyor Speed Control under Payload Change: Tuning, Scheduling and Windup](notebooks/04_motor_control/11_conveyor_speed.ipynb) | Intermediate | 22/22 checks passed |
| 12 | [Model Predictive Control of a Conveyor: Horizon, Constraints and What It Buys over PI](notebooks/06_advanced_control/12_mpc.ipynb) | Advanced | 14/14 checks passed |

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
