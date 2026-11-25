# Learning roadmap

Eight levels, from engineering fundamentals to integrated projects. **Study hours are recommendations
based on the author's judgement for a learner with the stated prerequisites. They are not measured
facts, and no learner has been timed.** Where the repository has no material yet, the level says so.

Notebook numbers refer to [notebooks.md](notebooks.md). Topic status is in [topic-taxonomy.md](topic-taxonomy.md).

| Level | Name | Recommended hours | Repository material |
|---|---|---|---|
| 0 | Engineering Fundamentals | 80 to 120 | None (prerequisite level; external courses) |
| 1 | Classical Control | 60 to 90 | Notebooks 01, 02, 03, 04 |
| 2 | Sequential and Industrial Control | 40 to 60 | Notebooks 05, 07 |
| 3 | Motor and Motion Control | 80 to 120 | Notebooks 06, 09, 10, 11 |
| 4 | Advanced Control | 80 to 120 | Notebook 12 (MPC only) |
| 5 | Intelligent Control | 50 to 80 | Notebook 08 (fuzzy PI only) |
| 6 | Industrial Integration | 60 to 100 | None |
| 7 | Advanced Engineering Projects | 150 or more | None |

## Level 0: Engineering Fundamentals
- **Prerequisites:** secondary-school algebra and trigonometry.
- **Topics:** calculus, linear algebra, ordinary differential equations, complex numbers, signals and
  systems, circuit analysis, basics of electrical machines and instrumentation, programming in Python.
- **Order:** mathematics, then circuits, then signals, then programming in parallel from the start.
- **Practical exercises:** simulate an RC circuit and a mass-spring-damper in Python and compare with the
  analytic step response.
- **Competencies:** write and read a first-order ODE model, integrate it numerically, plot and compare
  with a closed form.
- **Projects:** none. **Repository material:** none; use any standard university course.

## Level 1: Classical Control
- **Prerequisites:** Level 0.
- **Order:** 01 feedback fundamentals, 02 transfer functions and FOPDT identification, 03 PID, 04 tuning rules.
- **Topics:** open and closed loop, transfer functions, step response metrics, PID structure,
  derivative filtering, anti-windup, Ziegler-Nichols, Cohen-Coon, Tyreus-Luyben, IMC, relay tuning.
- **Practical exercises:** each notebook ends with exercises (section 15). Reproduce a table in
  `research/algorithm-comparisons/pid-tuning-rules.md` with a different plant.
- **Competencies:** identify an FOPDT model from a step test, tune a PI or PID, explain why a loop
  with delay loses stability, and recognise integrator windup.
- **Recommended projects:** Project 01 (in progress).
- **Not yet covered:** Bode and Nyquist analysis, root locus, Routh-Hurwitz.

## Level 2: Sequential and Industrial Control
- **Prerequisites:** Level 0 programming; Level 1 is helpful but not required.
- **Order:** 05 finite state machines, then 07 PLC scan cycle, timers and counters.
- **Topics:** Moore state machines, transition tables, interlocks, scan-cycle effects, TON/TOF/CTU,
  sequence control.
- **Competencies:** write a state machine with a complete transition table, predict a timer's behaviour
  under a given scan time.
- **Recommended projects:** Project 07 (in progress), Project 03 (planned).
- **Not yet covered:** ladder editors, vendor toolchains, HMI, safety-rated logic.

## Level 3: Motor and Motion Control
- **Prerequisites:** Level 1.
- **Order:** 06 DC motor, 09 BLDC six-step, 10 PMSM FOC, 11 conveyor speed.
- **Topics:** DC motor model, current/speed cascade, trapezoidal commutation, Hall alignment, Clarke
  and Park transforms, SVPWM, decoupling, voltage limit, belt conveyor under payload change.
- **Competencies:** build a cascade, explain what limits a current loop, explain torque ripple from a
  Hall offset, show why decoupling matters at high speed.
- **Recommended projects:** Projects 01, 02, 05, 06 (in progress).
- **Not yet covered:** induction motors, servo positioning, encoders as signals, VFD, synchronisation.

## Level 4: Advanced Control
- **Prerequisites:** Level 1 and linear algebra.
- **Material:** 12 linear MPC (horizon, constraints, comparison with PI and LQR).
- **Not yet covered:** state-space design, LQG, Kalman filters, adaptive and robust control.
- **Recommended projects:** Project 13 (in progress).

## Level 5: Intelligent Control
- **Prerequisites:** Level 1.
- **Material:** 08 fuzzy PI against PI and scheduled PI under a matched tuning budget.
- **Not yet covered:** neural control, reinforcement learning, evolutionary tuning beyond a fixed search.
- **Recommended projects:** Projects 04 and 15 (in progress).

## Level 6: Industrial Integration
No repository material. Planned topics: PLC and HMI, industrial communication (Modbus, PROFINET,
EtherCAT, OPC UA), drives, instrumentation, commissioning, diagnostics.

## Level 7: Advanced Engineering Projects
No repository material. Planned topics: robotics, AGV, predictive maintenance, multi-axis motion,
integrated systems. Project 14 is blocked on real data.
