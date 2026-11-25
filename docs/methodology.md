# Methodology

## What counts as a result
A result is a number produced by code in this repository, stored in `experiments/results/<id>.json`
together with the configuration that produced it. A statement in a notebook that has no such number
behind it is a claim and is worded as one.

## Checks
Each experiment carries automated checks. A check compares the code against something it does not
share an implementation with: a closed-form solution, a conservation law, an exact discretisation
(`scipy.signal`), a second solver, or an analytic floor.

When a check fails, the cause is one of three things: the test assumed something false, the library has
a defect, or the reference was misapplied. Tolerances are not loosened to make a check pass. Where a
threshold had to be chosen after seeing data, the notebook says so. Examples recorded in the notebooks:
the rate-constrained MPC check uses a KKT certificate because an interior-point solver is not accurate
enough to serve as an equality reference, and a false QP failure at a long horizon led to a library fix
(projected-gradient stopping criterion) and a unit test.

## Validation levels
| Level | Meaning | Present here |
|---|---|---|
| Unit test | A library function against a hand-derived value | Yes |
| Software simulation | Closed-loop run of a model with checks | Yes, 12 experiments |
| Controller in the loop | Real controller code against a simulated plant | No |
| Hardware in the loop | Real hardware signals | No |
| Physical hardware | Real machine | No |

## Statistics
Most experiments are deterministic. Where noise or a random search is used, the seed and the number of
trials are recorded in the result file. Results from a single seed are labelled as such.

## Comparisons between algorithms
A comparison is only reported with the same plant, the same scenario, the same actuator limits and,
for tuned controllers, the same tuning budget. A difference that disappears at matched control effort
is reported as such (notebook 12 shows the PI and MPC gap shrinking to about 4 % at matched effort).

## Reference verification
See `research/source-register.md`. A reference is "verified" only if its bibliographic record was found
on a publisher, index or library page. Full texts were not read. Nothing numeric is taken from a paper.

## Limits of this method
All plants are illustrative. Passing checks show internal consistency, not agreement with a real machine.
