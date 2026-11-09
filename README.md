# Optimization-Based Variable Impedance Control of a TiltHex in Wall Contact

Course project for *Numerical Optimization for Control* at Politecnico di Milano: optimization-based
variable impedance control for a fully actuated tilted hexarotor (TiltHex) whose end-effector is in contact
with a wall. It builds on the admittance filter (AF) and wrench observer (WO) of a Gazebo lab setup.
**Status: the measurement and infrastructure phase is complete; the optimization formulation is not yet
fixed, and no variable impedance controller exists yet.**

The detailed reports in [`noc/results/`](noc/results/) are in Turkish. This README summarizes them in English.
Verified inputs for the optimization, with sources: [`README_summary.md`](noc/results/README_summary.md).

**Two gains are easy to confuse:** `Kpxy` = 5 is the `uavpos` position gain; `K_x` = 10 N/m is the AF
stiffness in x (AF mass 5 kg, `B = 2·sqrt(m·K)`).

## Repository layout

```
noc-ws/                       (container: /shared-workspace)
├── assets/                   README diagrams from the original lab
├── gazebo/models/            mrsim-tilthex (lab model), noc-tilthex (adds the end-effector F/T sensor)
├── gazebo/worlds/            hexa-fa-wall-world (lab), noc-wall-world (loads noc-tilthex)
├── src/06a-physical-interaction/   original lab 6a, not modified
├── noc/common/               parameters, genomix readers, 6a setup fix, flight and safety helpers, F/T parser
├── noc/config/               AF-on baseline and pre-registered gate criteria
├── noc/experiments/          sim_noc.sh and the F/T logger
├── noc/tests/                flight experiments (t00–t04) and host unit tests (test_*.py)
├── noc/analysis/             log readers and per-experiment analysis
├── noc/results/              reports (Turkish), metrics and plots
└── logs/                     raw logs, gitignored
```

![Software architecture](assets/software_architecture.png)

Original lab architecture, from the original repository; NOC additions are under `noc/`.

## Requirements and setup

- Docker image `art/tk3lab:ionic-0.2` and the lecturers' release `r-1.3` scripts in `~/tk3lab/releases/r-1.3/scripts`.
- Host: Python 3 with the packages in [`requirements.txt`](requirements.txt), in `.venv/` (analysis and unit tests only).

`tk3lab-run` mounts `~/tk3lab-ws`. Create a launcher that mounts `~/noc-ws` as `/shared-workspace`:

```bash
cd ~/tk3lab/releases/r-1.3/scripts
sh tk3lab-load ~/tk3lab/docker-images/dimg-0.2/tk3lab-ionic-0.2.tar.gz   # once, if the image is not loaded
cp tk3lab-run tk3lab-run-noc
sed -i 's#TK3WS_DIR="${HOME}/tk3lab-ws"#TK3WS_DIR="${HOME}/noc-ws"#' tk3lab-run-noc
sh tk3lab-run-noc -g ionic -v 0.2
```

Open the noVNC desktop at <http://localhost:6080>.

Host environment and unit tests (no Docker needed):

```bash
cd ~/noc-ws
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m unittest discover -s noc/tests -t .
```

## Running experiments

**Which simulation script.** Both start the same components; only the world differs.

| Script | World / model | Used by |
|---|---|---|
| [`simulation.sh`](src/06a-physical-interaction/simulation.sh) | `hexa-fa-wall-world`, `mrsim-tilthex`, no F/T sensor | t01, t02, t03 |
| [`sim_noc.sh`](noc/experiments/sim_noc.sh) | `noc-wall-world`, `noc-tilthex`, F/T sensor on `/noc/ee_ft` | t04 |

**Terminal 1**, in a noVNC desktop terminal. Then press **Play** in the Gazebo window.

```bash
cd /shared-workspace/src/06a-physical-interaction && sh simulation.sh   # t01, t02, t03
cd /shared-workspace && sh noc/experiments/sim_noc.sh                   # t04
```

**Terminal 2**, inside the container. **Start Python only after Gazebo is up and Play is pressed.**
`--check` runs setup, start and preflight without flying.

```bash
cd /shared-workspace
# run one line, with the simulation that matches the test
python3 -m noc.tests.t01_af_on      --check && python3 -m noc.tests.t01_af_on
python3 -m noc.tests.t02_live_k     --check && python3 -m noc.tests.t02_live_k
python3 -m noc.tests.t03_k_ramp     --check && python3 -m noc.tests.t03_k_ramp
python3 -m noc.tests.t04_ft_check   --check && python3 -m noc.tests.t04_ft_check
```

| Test | What it does | Logs (`logs/noc/`) | Analysis (`noc.analysis.`) | Output (`noc/results/`) |
|---|---|---|---|---|
| [t01](noc/tests/t01_af_on.py) | 6a contact scenario with the WO and AF active | `t01_af_on/` | `a01_af_on` | `01_af_on/` |
| [t02](noc/tests/t02_live_k.py) | steps `K_x` live in contact: 10→12→15→20→15→10, then 200 unchanged calls | `t02_live_k/` | `a02_live_k` | `02_live_k/` |
| [t03](noc/tests/t03_k_ramp.py) | ramps `K_x` 10→20→10 at 10 N/m/s in contact | `t03_k_ramp/` | `a03_k_ramp` | `03_k_ramp/` |
| [t04](noc/tests/t04_ft_check.py) | free hover, steady contact, free hover with the F/T sensor recording | `t04_ft_check/` | `a04_ft_check` | `04_ft_check/` |

Raw logs are gitignored, so analysis needs a local run. On the host:

```bash
cd ~/noc-ws
.venv/bin/python -m noc.analysis.a04_ft_check          # same pattern for a01_af_on, a02_live_k, a03_k_ramp
.venv/bin/python -m noc.analysis.a04b_ft_supplement logs/noc/t04_ft_check noc/results/04_ft_check/supplement.json
.venv/bin/python -m noc.analysis.a04c_impulse_origin   # writes noc/results/04b_impulse_origin/
```

`src/06a-physical-interaction/` is never edited. Flight experiments load it through
[`setup_06a.py`](noc/common/setup_06a.py), which adds the missing `rotor_measure` connection.

## Key results so far

**Type tags.** *measured (F/T)*: Gazebo force/torque sensor, independent of the control chain. *measured (log)*:
logged or read-back value. *estimate*: force from the control chain (`uavpos fx` or WO `efx`). *computed*: derived
offline from a model or logs. *gate*: result on criteria committed before the flight.

| Result | Value | Type | Report |
|---|---|---|---|
| Gate A, live `K_x` steps (t02) | **FAILED**: 4 of 5 steps set the jump flag; other criteria passed | gate | [`02_live_k.md`](noc/results/02_live_k.md) |
| `K_x` applied live | 205/205 calls read back; 6100/6100 cycles on target | measured (log) | [`02_live_k.md`](noc/results/02_live_k.md) |
| Force-command jump per `K_x` step | 0.064–0.143 N | measured (log) | [`02_live_k.md`](noc/results/02_live_k.md) |
| Gate A2, `K_x` ramps (t03) | **PASSED** | gate | [`03_k_ramp.md`](noc/results/03_k_ramp.md) |
| Force-command step per ramp call, 10 N/m/s | median 0.0053 N, max 0.034 N | measured (log) | [`03_k_ramp.md`](noc/results/03_k_ramp.md) |
| Gate B1, F/T sensor (t04) | **PASSED** (C1–C6) | gate | [`04_ft_check.md`](noc/results/04_ft_check.md) |
| Steady contact force, `K_x` = 10 | 0.616 N (95 % CI 0.540–0.697) | measured (F/T) | [`04_ft_check.md`](noc/results/04_ft_check.md) |
| Series-model force, same point | 0.613 N | computed | [`04_ft_check.md`](noc/results/04_ft_check.md) |
| WO `efx` / `-uavpos fx`, same contact | −0.650 N / −0.608 N | estimate | [`04_ft_check.md`](noc/results/04_ft_check.md) |
| Series model vs `fx`, `K_x` = 10–20 | error −6.5 % to −0.4 % | computed vs estimate | [`README_summary.md`](noc/results/README_summary.md) |
| 04b: physics engine | DART; does not read `<ode><kp>/<kd>/<min_depth>` | source code and installed binary | [`04b_impulse_origin.md`](noc/results/04b_impulse_origin.md) |
| 04b: steady-contact pulses | about 29 Hz, mostly 1–2 physics steps, median force ≈ 0 | measured (F/T) | [`04b_impulse_origin.md`](noc/results/04b_impulse_origin.md) |
| 04b: first impact | about 350 N | measured (F/T) | [`04b_impulse_origin.md`](noc/results/04b_impulse_origin.md) |
| 04b: tip separation from the wall | up to 2.3 mm | measured (log) | [`04b_impulse_origin.md`](noc/results/04b_impulse_origin.md) |
| Force upper bound, `d0` = 0.184 m, `Kpxy` = 5 | 0.92 N | computed | [`README_summary.md`](noc/results/README_summary.md) |
| `set_af_parameters` p95 | 6.01 ms (t02), 8.18 ms (t03) | measured (log) | [`README_summary.md`](noc/results/README_summary.md) |
| Real-time factor | 0.68 over the t04 flight, 0.645 in contact | measured (log) | [`README_summary.md`](noc/results/README_summary.md) |

## Measurement rule

**Contact force is the F/T normal force (world-x component minus free-flight bias), aligned on simulation time
and filtered with a 0.5 s moving average. Raw F/T is for diagnosis only.**

DART does not read `<ode><kp>/<kd>/<min_depth>`, so the contact is rigid and the instantaneous F/T signal is
impulsive ([04b](noc/results/04b_impulse_origin.md)). The filtered mean agrees with the series model: 0.616 N
measured, 0.613 N computed, in t04.

This rests on one flight at one operating point (`K_x` = 10, `d0` = 0.184 m), resolved to about ±13 %. The
0.616 N uses a post-hoc 0.1 s time-alignment envelope. The 2 s envelope pre-registered for Gate B1 places F/T
about 0.5 s early at RTF 0.64 and gives 0.574 N ([`04_ft_check.md`](noc/results/04_ft_check.md) §2, §4).

## Fixed issues and known issues

**Fixed** (applied in `noc/`, verified on a run):

- **WO and AF inactive:** 6a `setup()` never connected `uavatt.rotor_measure`. Fixed in `setup_06a.py` ([00b](noc/results/00b_diagnosis.md), [01](noc/results/01_af_on.md)).
- **Recorder blind during motion:** `maneuver.goto` blocked; now uses `ack=True` ([01](noc/results/01_af_on.md)).
- **Input-port reads failed:** only output ports and attribute services are read ([00b](noc/results/00b_diagnosis.md)).
- **6a `stop()` overwrote logs:** `LOG_DIR` is redirected. The raw logs behind `00_baseline.md` were lost before this fix.
- **No contact-force measurement:** F/T sensor on `ee_joint` ([04](noc/results/04_ft_check.md)).
- **`K_x` steps made the force command jump:** `K_x` now changes by ramp ([03](noc/results/03_k_ramp.md)). Gate A stays failed by its definition.
- **False preflight failure before the first goto:** those ports are "pending" until then ([03](noc/results/03_k_ramp.md)).
- **Safety v1 tripped at landing:** the v2 external-wrench check runs only while flying; t02–t04 had no trip.
- **Wall stiffness unknown:** DART ignores the `kp`/`kd` values, so `params.K_WALL` is not applied ([04b](noc/results/04b_impulse_origin.md) §5).

**Known issues:**

- **Pulse origin not definitively separated.** 04b attributes the pulse shape to DART rigid contact and the mean force to control, but no flight varied the control to test this. The optimization will use the mean force.
- **The tip leaves the wall** by up to 2.3 mm in steady contact; **first impact is about 350 N** ([04b](noc/results/04b_impulse_origin.md)).
- **The series model is checked against F/T at one point only** (`K_x` = 10). `K_x` = 12–20 is compared only with `fx`; `Kpxy` was never varied.
- **The lab requirement |Fx| ≥ 2 N is not met** (0.616 N measured).
- **`maneuver` runs on the wall clock and RTF is about 0.64,** so commanded speeds are about 1.56 times larger in simulation time (computed).
- **Settling time is not reliably measured** ([02](noc/results/02_live_k.md) §4, [03](noc/results/03_k_ramp.md) §5).
- **`B` is fixed** at 14.14 N·s/m when `K_x` changes, so ζ falls from 1.0 to 0.71 at `K_x` = 20 (computed).
- **WO and `fx` disagree by 0.0–0.1 N,** varying from hold to hold; their lag behind F/T could not be determined.
- **`K_x` is tested only in 10–20 N/m and ramps only at 10 N/m/s.** Kvxy and `Kpxy` sweeps were not done.
- **Other:** `phynt.log` missed 7.5 % of records in t01; `fx` noise is higher in t03 than t02 (cause unknown); model inconsistencies are listed in [`CLAUDE.md`](CLAUDE.md).

## Next steps

Open decisions for the optimization problem. None of these is decided.

- **`Kpxy` and the `K_x` series limit.** In the series model `F = d0 / (1/Kpxy + 1/K_x)`, raising `K_x` alone cannot exceed `d0·Kpxy` (0.92 N today, computed). Should `Kpxy` or `d0` also change, and would they be decision variables or fixed parameters? Neither was varied in flight.
- **Force target range.** Is the target 2 N or a range the current setup can reach? For 2 N (computed): at `Kpxy` = 5, `d0` = 0.60 m for `K_x` = 10 and 0.50 m for `K_x` = 20; at `d0` = 0.184 m, `Kpxy` = 23.8 for `K_x` = 20, and `K_x` = 10 has no solution. See [`README_summary.md`](noc/results/README_summary.md).
- **Static or dynamic problem.** Choose `K_x` for a steady force from the series model, or optimize a `K_x(t)` trajectory under the ramp constraint? A dynamic problem needs settling behavior, which is not reliably measured yet, and a decision on whether `B` changes with `K_x`.

## Credits and license

Course: Numerical Optimization for Control, Prof. Lorenzo Fagiano, Politecnico di Milano, 2026–27.

Team: `<TODO>`

Origin: [ismailcemyilmaz/noc_project](https://github.com/ismailcemyilmaz/noc_project). Upstream:
[gizemdogafiliz/Aerial-Robotics-FromHoverToContact](https://github.com/gizemdogafiliz/Aerial-Robotics-FromHoverToContact),
the lab work of **Gizem Doğa Filiz** on which this project is built. Labs 1–5b are in the git history
(`git show b39a541:src/<lab>/...`).

Intellectual credit for the lab design, course material, `tk3lab` Docker image and `telekyb3` belongs to:

- Dr. **Marco Tognon** and Dr. **Gianluca Corsini**, RAINBOW Team, Inria / IRISA / Université de Rennes, lecturers of *Aerial Robotics - Advanced Topics in Automation and Control Engineering* (Politecnico di Milano, 2025–26).
- The authors of [`telekyb3`](https://git.openrobots.org/projects/telekyb3) and its [GenoM3](https://www.openrobots.org/wiki/genom3) components (LAAS-CNRS, IRISA-Inria, University of Twente, University College London), in particular **Anthony Mallet** (LAAS-CNRS). No component was implemented here.
- The container launcher `tk3lab-run`, from which `tk3lab-run-noc` is derived, is by Gianluca Corsini, © 2026 IRISA (CNRS/INRIA), BSD-style license.

The wrench observer and admittance filter design follows:

> T. Tomić, C. Ott, and S. Haddadin, "External Wrench Estimation, Collision Detection, and Reflex Reaction for Flying Robots", *IEEE Trans. on Robotics*, vol. 33, no. 6, pp. 1467–1482, Dec. 2017. [DOI](https://doi.org/10.1109/TRO.2017.2750703)

> A. De Luca, A. Albu-Schäffer, S. Haddadin, and G. Hirzinger, "Collision Detection and Safe Reaction with the DLR-III Lightweight Manipulator Arm", IROS 2006. [DOI](https://doi.org/10.1109/IROS.2006.282053)

Released under the MIT License. Original lab code © 2026 Gizem Doğa Filiz; NOC project additions (`noc/`, `gazebo/models/noc-tilthex`, `gazebo/worlds/noc-wall-world.world`) © 2026 the project team. See [`LICENSE`](LICENSE).
