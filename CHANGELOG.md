# Changelog

## 0.1.0 (2026-11-25), first build

Added
- `control_lab` library: LTI and FOPDT plants, nonlinear tank, belt conveyor, DC, BLDC and PMSM motors,
  PID with anti-windup, tuning rules and relay auto-tuner, cascade, Mamdani fuzzy, FSM, PLC function
  blocks, FOC, linear MPC, closed-loop simulation, experiment tracking with a JSON schema.
- 12 research notebooks (15 sections each), executed from a clean kernel: 179 automated checks, all passed.
- Research registers, algorithm comparisons, research gaps, standards register (19 references, 2 standards).
- Dataset catalog with one synthetic example; no measured data.
- Preview website (11 sections) driven by `experiments/index.json`; jsdom smoke test (140 assertions).
- Taxonomy of 283 topics and a 15-project catalog with honest statuses.
- Tests: 151 (unit, integration, simulation, notebook structure). Ruff clean.
- GitHub Actions workflows (written, **not run**).

Measured on the build machine (2 CPU cores, Python 3.13): executing all 12 notebooks took 3 min 31 s;
the slowest is notebook 08 (about 114 s).

Known gaps
- No hardware validation of any kind.
- CI has not run on GitHub; the Python 3.11 and 3.12 matrix entries were not exercised locally (3.13 only).
- No literature from 2022 onward; Cohen-Coon and Tyreus-Luyben references only partly verified; the
  source of the Smith two-point identification method was not located.
- IEC 61131-3:2013 has a newer edition (2025) that was not examined; the ISO 5048 record shows
  conflicting review dates on two language pages.
- 200 of 283 topics are Planned; 6 of 15 projects are Planned and Project 14 is Blocked.
