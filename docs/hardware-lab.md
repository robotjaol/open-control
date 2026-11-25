# Hardware lab

**Status: no hardware experiment has been performed for this repository. Every hardware validation
status is "Not validated".** This page describes what a hardware validation would have to contain, so
that a future contribution can be judged against it. It is not a record of work done.

## What a hardware validation must provide
1. Hardware list with model numbers and ratings, and the firmware version (`hardware_configuration`
   and `firmware_version` in the result schema).
2. Measured data in `datasets/` with a catalog entry of kind `measured_experiment`: instruments,
   calibration, sampling time, operating conditions, raw files and their checksums.
3. The identical scenario run in simulation with the identified parameters, and both results stored.
4. A statement of agreement and of every disagreement, with the metric used. A model that fits one
   operating point is not evidence about another.
5. `experiment_type` set to `hardware_in_the_loop` or `physical_hardware`, with the limitations of the setup.

## Safe-practice reminders (general, not a safety design)
- Work with energised mains equipment only with qualified supervision and local regulations.
- Use a current-limited supply and a hardware stop that does not depend on the software under test.
- A software emergency-stop state (as in notebook 05) is not a safety function. Safety functions need a
  separate design to the applicable standard (for example IEC 61508 and ISO 13849-1, neither verified here).
- Start motor tests unloaded, at low voltage, and increase one variable at a time.

## Planned hardware targets (not started)
DC motor with encoder; BLDC with Hall sensors; PMSM drive; small belt conveyor; a PLC. No part has been selected.
