# Simulation lab

The library gives a small closed-loop protocol so that any controller can be run against any plant.

| Object | Interface |
|---|---|
| Plant | `y` (measured output), `step(u, d)`, `reset()` |
| Controller | `update(r, y[, u_ff])` returning the command |
| Loop | `run_closed_loop(plant, controller, ref, Ts, disturbance=, noise_std=, quant_step=, seed=, ff=, actuator=, record=)` |
| Actuator | `Actuator(u_min, u_max, rate, u0)`: saturation and slew limit |

## Plants in the library
FOPDT and LTI plants (`models.lti`), nonlinear tank (`models.tank`), belt conveyor with payload events
(`models.conveyor`), DC motor, BLDC (trapezoidal back-EMF) and surface-magnet PMSM (`motors`).
Integration is RK4 with sub-steps; every model is verified in unit tests against an exact response.

## Controllers
PID (positional, derivative filter, clamping and back-calculation anti-windup, bumpless tracking),
tuning rules and relay auto-tuner, cascade, Mamdani fuzzy and fuzzy PI, finite state machine, PLC
function blocks, field-oriented current control, linear MPC (condensed QP).

## Adding an experiment
1. Create `tools/specs/sNN_name.py` with `META` and `build()` returning sections 1 to 15.
2. Add it to `tools/specs/__init__.py`.
3. In section 10 to 12 compute the metrics, define checks against an independent reference, and save
   an `ExperimentRecord`.
4. `python tools/build_notebooks.py && python tools/run_notebooks.py sNN_name`.
5. Rebuild the derived files (see getting-started).

## Rules
- Parameters are illustrative unless a dataset section says otherwise.
- A check that fails is investigated; it is not loosened to pass.
- Timing measurements (such as QP solve time) are specific to the machine that ran them.
