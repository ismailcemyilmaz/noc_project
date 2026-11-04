# NOC - Variable impedance control of a TiltHex in contact with a wall

> Built on the *Aerial Robotics Laboratory* of the course *Advanced Topics in Automation and Control
> Engineering* (Politecnico di Milano, 2025–26). <br>
> **Lecturers:** Dr. Marco Tognon, Dr. Gianluca Corsini - *RAINBOW Team, Inria / IRISA / Université de Rennes*

**Goal.** Variable impedance control for the fully-actuated tilted hexarotor (TiltHex) while its
end-effector is in contact with a wall. The starting point is lab assignment 6a (physical interaction:
`phynt-genom3` admittance filter + wrench observer, Gazebo wall world).

**Current status (2026-11-04): measurement and infrastructure phase.** The optimisation objective is not
fixed yet. What exists today:

- a diagnosis of the unmodified 6a pipeline (`noc/results/00b_diagnosis.md`);
- a verified AF-on reference run, t01 (`noc/results/01_af_on.md`, config `noc/config/af_on_baseline.json`);
- a reusable, tested flight path in `noc/` (setup fix, non-blocking goto, safety monitor, output-port
  readers, log analysis).

No variable impedance controller exists yet. The original 6a work is kept unmodified in
`src/06a-physical-interaction/`. Labs 1–5b were removed from the tree; they are in git history
(`git show b39a541:src/<lab>/...`).

The work is built on top of the [**`telekyb3`**](https://git.openrobots.org/projects/telekyb3) framework, an
open-source collection of GenoM3 components for multi-rotor UAVs developed by **LAAS-CNRS, IRISA-Inria,
University of Twente** and **University College London**. All control / estimation / interaction
components used here (`uavpos-genom3`, `uavatt-genom3`, `pom-genom3`, `optitrack-genom3`,
`rotorcraft-genom3`, `maneuver-genom3`, `phynt-genom3`, `mrsim-gazebo`) are part of `telekyb3` - full credit
for their design and implementation goes to the original authors and to the
[RAINBOW team](https://team.inria.fr/rainbow/).

**Terminology used throughout.** Two different gains are easy to confuse:

| Symbol | Where | Value | Meaning |
|---|---|---|---|
| `Kpxy`, `Kvxy` | `uavpos` position controller | 5, 7 | position / velocity gains; in steady contact `fx = −Kpxy·e_x − Kvxy·e_vx` (00b Q4) |
| `K_x` | `phynt` admittance filter | 10 N/m | admittance stiffness in x (`af_K[0]`), with AF mass 5 kg and `B = 2·sqrt(m·K)` |

All forces in this README are **estimates** (wrench observer output or the uavpos force command). There is
no contact-force measurement in the simulation (no contact or F/T sensor).

---

## Table of Contents

1. [Framework & background](#framework--background)
2. [Repository layout](#repository-layout)
3. [Requirements and container](#requirements-and-container)
4. [Verified workflow: t01 (AF on)](#verified-workflow-t01-af-on)
5. [Using 6a unmodified: known read problems and the noc path](#using-6a-unmodified-known-read-problems-and-the-noc-path)
6. [Fixed issues](#fixed-issues)
7. [Open issues](#open-issues)
8. [Original 6a assignment](#original-6a-assignment)
9. [Acknowledgments](#acknowledgments)
10. [References](#references)

---

## Framework & background

The lab assignments use the following open-source robotics stack, all developed primarily at
LAAS-CNRS / IRISA-Inria:

| Component | Purpose | Reference |
|---|---|---|
| [`telekyb3`](https://git.openrobots.org/projects/telekyb3) | Modular software framework for multi-rotor UAVs | open-source, LAAS-CNRS / IRISA-Inria / Twente / UCL |
| [GenoM3](https://www.openrobots.org/wiki/genom3) | Real-time component-based middleware abstraction | LAAS-CNRS, Anthony Mallet |
| [robotpkg](http://robotpkg.openrobots.org/) | Source-based package manager for robotics software | LAAS-CNRS |
| [Gazebo](https://gazebosim.org/) | 3-D physics simulator | Open Source Robotics Foundation |
| [SDFormat](https://sdformat.org/) | XML format used to describe Gazebo robots / worlds | OSRF |
| [Genomix](https://git.openrobots.org/projects/genomix) | HTTP/JSON bridge that lets Python clients call GenoM3 services | LAAS-CNRS |
| [`mrsim-gazebo`](https://git.openrobots.org/projects/mrsim-gazebo) | Multi-rotor Gazebo plugin (motors + IMU) | LAAS-CNRS |
| [`optitrack-gazebo`](https://git.openrobots.org/projects/optitrack-gazebo) | Mocap simulation plugin | LAAS-CNRS |

The GenoM3 components used directly in this repository - every `<name>-genom3` referenced below - are part
of the `telekyb3` collection. Their full documentation lives at
`https://git.openrobots.org/projects/<name>-genom3/pages/README`. None of them was implemented by me -
my work is to *use* and *parameterise* them through Python clients, plus to write the dynamics simulators,
custom controllers, plotting, and SDF model edits required by each assignment.

### High-level architecture

![Software architecture](assets/software_architecture.png)

User Python clients drive [`genomixd`](https://git.openrobots.org/projects/genomix) over HTTP/JSON.
`genomixd` exposes the full set of `telekyb3` GenoM3 components (state estimation, controllers, observers,
trajectory generator) sitting on a real-time middleware bus. The bottom layer is Gazebo with the
`mrsim-gazebo` plugin, which provides the simulated motors, IMU, and contact dynamics - the equivalent
of the real flight controller / ESC / IMU stack on the physical platform.

---

## Repository layout

```
noc-ws/                                      (container: /shared-workspace)
├── README.md, LICENSE, CLAUDE.md
├── assets/                                  ← README diagrams
├── gazebo/
│   ├── models/mrsim-tilthex/                ← TiltHex SDF with the EE bar + sphere tip (r = 0.04 m)
│   └── worlds/hexa-fa-wall-world.world      ← TiltHex + thin static wall at x = 2.0 m
├── src/06a-physical-interaction/            ← original lab 6a, NOT modified (simulation.sh, model_hexa_fa.py, plot_06a.py)
├── noc/
│   ├── common/      params.py (physical parameters with sources), telekyb_io.py (output-port readers),
│   │                setup_06a.py (6a setup + rotor_measure fix), flight.py (goto, safety, recording helpers)
│   ├── config/      af_on_baseline.json (reference controller / AF / WO configuration, run t01)
│   ├── tests/       experiments (t00_*, t01_af_on.py) and host unit tests (test_*.py, fake_genomix.py)
│   ├── analysis/    log readers and per-experiment analysis (baseline_06a, a00_diag, a01_af_on)
│   └── results/     reports: 00_baseline.md, 00b_diagnosis.md, 01_af_on.md (+ metrics/plots)
└── logs/                                    ← raw logs, gitignored (logs/noc/<experiment>/)
```

`mrsim-rotor` (used six times by the TiltHex SDF) is not in this repo; it comes from the tk3lab image
(`/opt/openrobots`). `simulation.sh` prepends `/shared-workspace/gazebo/models` to `GZ_SIM_RESOURCE_PATH`
so the local `mrsim-tilthex` (with the EE bar) wins over the image copy.

---

## Requirements and container

- The `tk3lab` Docker image provided by the lecturers (`art/tk3lab:ionic-0.2`: telekyb3, GenoM3,
  Gazebo Ionic, noVNC desktop) and the lecturers' release scripts in `~/tk3lab/releases/r-1.3/scripts`.
- Host: Python 3 with numpy and matplotlib in `.venv/` (analysis and unit tests only).

The lecturers' `tk3lab-run` mounts `~/tk3lab-ws`. This project needs `~/noc-ws` mounted instead.
`tk3lab-run-noc` is **not** in this repo; create it once from `tk3lab-run` (script by Gianluca Corsini,
IRISA) by changing only the workspace directory:

```bash
# (host)
cd ~/tk3lab/releases/r-1.3/scripts
sh tk3lab-load ~/tk3lab/docker-images/dimg-0.2/tk3lab-ionic-0.2.tar.gz   # once, if the image is not loaded
cp tk3lab-run tk3lab-run-noc
sed -i 's#TK3WS_DIR="${HOME}/tk3lab-ws"#TK3WS_DIR="${HOME}/noc-ws"#' tk3lab-run-noc
sh tk3lab-run-noc -g ionic -v 0.2
# open http://localhost:6080 → Linux desktop; /shared-workspace inside the container is ~/noc-ws
```

Host unit tests (no Docker):

```bash
cd ~/noc-ws && .venv/bin/python -m unittest discover -s noc/tests -t .
```

---

## Verified workflow: t01 (AF on)

`noc/tests/t01_af_on.py` runs the 6a contact scenario (same waypoints and timing as 6a) through the noc
path: `setup_06a.setup()` (6a `setup()` + the `rotor_measure` connection), non-blocking `flight.goto`,
a 50 Hz recorder of output ports, and the `flight.Safety` monitor (retreat + land + `stop()` on a trip).

**T1** - in a noVNC desktop terminal (`simulation.sh` needs the desktop for Gazebo):

```bash
cd /shared-workspace/src/06a-physical-interaction && sh simulation.sh
```

When the Gazebo window opens, press **Play**.

**T2** - a second terminal inside the container:

```bash
cd /shared-workspace
python3 -m noc.tests.t01_af_on --check   # setup + start + preflight + stop, no flight → logs/noc/t01_check/
python3 -m noc.tests.t01_af_on           # flight (80 s) → logs/noc/t01_af_on/
```

`--check` prints `[check] reads ALL OK; wrench_measure.force valid: True; ...` and exits 0 only when every
preflight read succeeds and `wrench_measure.force` (needs the `rotor_measure` connection) and
`rotor_measure.velocity` are valid; details in `logs/noc/t01_check/check.json`.

**H** - analysis on the host:

```bash
cd ~/noc-ws && .venv/bin/python -m noc.analysis.a01_af_on   # → noc/results/01_af_on/ (metrics.json, compare.png)
```

Run outputs (`logs/noc/t01_af_on/`): `t01_af_on.npz` (all samples + meta/events JSON), `meta.json`
(git rev, safety version, abort reason), `events.json` (goto call times), `snapshots.json` (phynt
parameters/log_info), `preflight.json`, and the component logs copied by 6a `stop()` (redirected `LOG_DIR`).
The reference run t01 (2026-11-04, git `0e9a210`, safety version 1) and its npz sha256 are recorded in
`noc/config/af_on_baseline.json`. Neither the scripts nor the simulator seed are randomised by us; the
simulator seed is not recorded.

---

## Using 6a unmodified: known read problems and the noc path

`src/06a-physical-interaction/` is kept exactly as submitted. Run as-is (`python3 -i model_hexa_fa.py`,
`simulation()`), it has these problems (`noc/results/00b_diagnosis.md`):

| Problem in 6a | Effect | noc path |
|---|---|---|
| `setup()` never connects `uavatt.rotor_measure` | `wrench_measure.force` is None → WO writes nothing → AF deviation 0, `phynt.log` empty | `noc.common.setup_06a.setup(m)` |
| `maneuver.goto(...)` without `ack` | each call blocks for the whole motion (2.0–5.1 s); no samples during motion | `noc.common.flight.goto(m, args)` (`ack=True`) |
| `read_desired_phynt()` / `read_nominal()` read `d['pos']['x']`; `pos` is None in static phases | `TypeError` swallowed, zeros written (`simulation_data.npz`: `p_nom` 4000/4000 zero) | `noc.common.telekyb_io` readers mark None explicitly |
| `read_state()` calls `pom.frame()` without the port name | fails every call, `pos`/`vel` zero in the npz | `telekyb_io.read_pom` (`pom.frame('robot')`) |
| `read_external_wrench()` calls `get_wo_data`, which does not exist | `f_ext` zero in the npz | `telekyb_io.read_external_wrench` (output port `external_wrench`) |
| input ports (e.g. `phynt.reference`) read from Python | genomix `TypeError` (needs a path) | read only output ports / attribute services (`get_reference`) |
| `stop()` copies component logs to a fixed `LOG_DIR` | overwrites `logs/06a-physical-interaction/gazebo/` | `setup_06a.load_06a(log_dir)` redirects to `logs/noc/<exp>/` |
| `plot_06a.py` "contact force" | computed `K_WALL × penetration`, not a measurement | `noc/analysis/` reports label every force as estimate |

---

## Fixed issues

Only fixes that are applied in `noc/` and verified on a run are listed. `src/06a` is not modified.

**1. Wrench observer and admittance filter inactive.**
- *Problem:* `phynt/external_wrench` never written (t00: 0 % valid), AF deviation exactly 0, `phynt.log`
  header only (`log_info` total 0).
- *Cause:* 6a `setup()` does not connect the `uavatt` input port `rotor_measure`, so
  `uavatt/wrench_measure.force` (the WO input) is always None.
- *Fix:* `noc/common/setup_06a.py`: after 6a `setup()`, verify the port (uavatt doc + live rotorcraft read),
  then `uavatt.connect_port({'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'})`.
- *Evidence (t01, `01_af_on.md` §1–4):* `wrench_measure.force` valid 0 % → 99.97 %, `external_wrench`
  0 % → 100 %; `phynt.log` 0 → 59 485 rows; steady-contact AF deviation 0 → −55.0 mm; the logged
  deviation is reproduced by the AF model driven by the logged WO force (RMS 2.1 mm, corr. 0.988).

**2. Recorder blind during motion.**
- *Problem:* no samples while the drone moves; up to 5.1 s gaps.
- *Cause:* `maneuver.goto` without `ack` blocks for the whole motion.
- *Fix:* `noc/common/flight.py` `goto(m, args)` calls `maneuver.goto(*args, ack=True)`.
- *Evidence:* goto call time 2.0–5.1 s (t00) → 0.8–7 ms (t01); recorder gaps > 100 ms 10 → 1 (the
  remaining one is the blocking `set_wo_zero`, 2.6 s).

**3. Input-port reads abort the run.**
- *Problem:* first t00_diag run stopped with genomix `TypeError` (missing `path`) on `phynt.reference()`.
- *Cause:* genomix input ports cannot be read without a path argument.
- *Fix:* `noc/common/telekyb_io.py` reads only output ports and attribute services
  (`phynt.get_reference()`, `uavatt.wrench_measure()`); `preflight()` records each source's status.
- *Evidence:* t00_check ALL OK; t01 `meta.json` `read_error_counts = {}` over the whole flight.

**4. 6a `stop()` overwrites earlier logs.**
- *Problem:* a run through 6a `stop()` replaced the 6a baseline logs in `logs/06a-physical-interaction/gazebo/`.
- *Cause:* 6a `LOG_DIR` is a fixed path.
- *Fix:* `setup_06a.load_06a(log_dir)` sets the module's `LOG_DIR` to `logs/noc/<exp>/` before any run.
- *Evidence:* after t01 (2026-11-04 17:21) the files in `logs/06a-physical-interaction/gazebo/` still have
  their 2026-11-03 timestamps.
- *What was lost before the fix:* the raw logs behind `noc/results/00_baseline.md`. Checked on
  2026-11-04: `logs/06a-physical-interaction/gazebo_play_late/` does not exist in this workspace, and
  of the 9 files in `noc/results/00_baseline/log_inventory.json` only `phynt.log` still matches its
  sha256 (it is a 75-byte header with no data rows). The other 8 (`pom.log`, `pom-measurements.log`,
  `uavpos.log`, `uavatt.log`, `maneuver.log`, `rotorcraft.log`, `opti.log`, `simulation_data.npz`) now
  hold the t00_probe run. Preserved: the derived metrics (`metrics.json`), plots and the inventory with
  sizes, row counts and sha256 of the lost files. `~/tk3lab-ws` was not searched.

---

## Open issues

- **Contact force is an estimate, not a measurement.** Steady contact in t01: uavpos force command
  `fx` 0.622 N, WO `f_ext,x` −0.606 N. Both come from the same control chain and are not independent.
  Measuring it needs a Gazebo contact or F/T sensor (the SDF has a "contact sensor" comment but no `<sensor>`).
- **Lab requirement |Fx| ≥ 2 N is not met** (≈ 0.62 N estimated). The 2 N figure in the original 6a
  write-up assumed a 500 N/m wall spring.
- **Wall stiffness: hypothesis, not verified.** The world sets ODE `<kp>500</kp>`, but `<physics
  type='unused'>` is started with `gz sim`; the default engine may ignore the ODE contact parameters.
  The 6a baseline data give an effective wall stiffness ≥ ~1.1·10⁴ N/m under stated assumptions
  (`00_baseline.md`); `params.K_WALL = 500` remains marked ASSUMPTION.
- **Safety monitor v2 not flight-tested.** The external-wrench check is now active only while flying
  (z > 0.5 m), because t01 (safety v1) tripped at landing on ground contact (WO |f| = 3.54 N, z = 0.237 m).
  v2 is covered by a unit test only.
- **`phynt.log` misses 7.5 % of records** (`log_info` miss 4 832 / 64 113 in t01).
- **Steady contact windows are short.** 1.9–3.5 s against an AF settling time of ≈ 4.1 s; the measured
  deviation (−55 mm) is 6–8 mm below the series-model prediction (61–63 mm).
- **Static-phase reference `pos` is None** (00b E2) in `maneuver.desired` / phynt reference; the AF output
  `phynt.desired` still carries pos and vel. Mechanism not verified.
- **Gains not explored.** Kvxy sweep not done (model inference: Kvxy contributes ≈ 0.02 N statically and
  does not set static stiffness). Live test of the admittance stiffness K_x is the next step.
- **Model inconsistencies** (`CLAUDE.md`): rotor tilt mapping (SDF ≈ 20° vs `rx = −21.2°, ry = −18.7°`)
  unverified; inertia is base-link only; `plot_06a.py` EE kinematics ignore the tip z offset.
- **6a read functions** (table above) are not fixed in 6a itself; use the noc path.

---

## Original 6a assignment

The submitted lab 6a, kept unmodified in `src/06a-physical-interaction/`: a fully-actuated hexarotor
with an L-shaped end-effector (a vertical 0.10 m bar, then a horizontal 0.6 m bar along body x ending in
a sphere tip of radius 0.04 m) makes contact with a thin static wall and slides the tip along a 1 m square.
The interaction is mediated by [`phynt-genom3`](https://git.openrobots.org/projects/phynt-genom3):

- a **wrench observer (WO)** that estimates the external wrench from the (known) rotor wrench and the
  measured drone state, following Tomić, Ott & Haddadin's design ([IEEE T-RO 2017](https://ieeexplore.ieee.org/document/8049322));
- an **admittance filter (AF)** that turns the maneuver-generated nominal trajectory into a compliant
  reference (mass–spring–damper response to the estimated wrench).

![6a data flow](assets/6a_dataflow.png)

Demo recording of the original run: https://github.com/user-attachments/assets/128ed5de-2f88-4ef9-a5f8-4a10dfc13c03

Parameters set by 6a `setup()` (recorded in `noc/config/af_on_baseline.json`): AF
`af_K = [10, 100, 100, 50, 50, 50]`, AF mass 5; WO `K = [1, 1, 1, 0, 0, 0]`, `fc = [20, 20, 20, 1, 1, 1]`;
drone mass 2.72 kg (2.3 kg base + 6 × 0.07 kg `mrsim-rotor`).

| t (s) | (x, y, z, yaw) | duration | description |
|---|---|---|---|
|  2 | (0,    0, 1.0, 0) | 5 | hover takeoff |
| 10 | (1.2,  0, 1.0, 0) | 4 | approach wall |
| 20 | - | - | WO calibration (`set_wo_zero`, 2 s) |
| 22 | (1.55, 0, 1.0, 0) | 2 | contact bottom-left |
| 30 | (1.55, 1, 1.0, 0) | 5 | slide Y |
| 38 | (1.55, 1, 2.0, 0) | 5 | slide Z |
| 46 | (1.55, 0, 2.0, 0) | 5 | slide Y |
| 54 | (1.55, 0, 1.0, 0) | 5 | slide Z, close square |
| 62 | (1.0,  0, 1.0, 0) | 4 | retract |
| 69 | (0,    0, 0,   0) | 5 | land |

The original plots in `src/06a-physical-interaction/plots_06a/gazebo/` come from the 6a run, in which the
WO and AF were inactive (Fixed issue 1). Their "wrench" and "contact" panels show the idealised
`K_WALL × penetration`, not a measured force.

---

## Acknowledgments

This repository is the result of work I carried out as a student on the *Aerial Robotics - Advanced Topics in Automation and Control Engineering* course (a.y. 2025–26). All
intellectual credit for the lab design, the course material, the `tk3lab` Docker image, and the
`telekyb3` software framework belongs to:

- Dr. **Marco Tognon** - *Inria, RAINBOW Team*
- Dr. **Gianluca Corsini** - *Inria / Université de Rennes, RAINBOW Team*
- The original authors of `telekyb3` and its constituent GenoM3 components, in particular **Anthony
  Mallet** (LAAS-CNRS) and the maintainers at IRISA-Inria, the University of Twente, and University
  College London.

The container launcher `tk3lab-run` (from which `tk3lab-run-noc` is derived, see
[Requirements and container](#requirements-and-container)) is by Gianluca Corsini, © 2026 IRISA (CNRS/INRIA),
BSD-style license. The lab assignments 1–5b that preceded this project are in the git history of this
repository (`git show b39a541:src/<lab>/...`).

The wrench observer / admittance filter design follows:

> T. Tomić, C. Ott, and S. Haddadin, *"External Wrench Estimation, Collision Detection, and Reflex
> Reaction for Flying Robots"*, IEEE Trans. on Robotics, vol. 33, no. 6, pp. 1467–1482, Dec. 2017.
> ([DOI](https://doi.org/10.1109/TRO.2017.2750703))

The momentum-based torque observer is from:

> A. De Luca, A. Albu-Schäffer, S. Haddadin, and G. Hirzinger, *"Collision Detection and Safe
> Reaction with the DLR-III Lightweight Manipulator Arm"*, IROS 2006.
> ([DOI](https://doi.org/10.1109/IROS.2006.282053))
---

## References

### Course material
- Course page (Politecnico di Milano): *055512 - Advanced Topics in Automation and Control Engineering*
- RAINBOW Team @ Inria: <https://team.inria.fr/rainbow/>

### Software
- `telekyb3`: <https://git.openrobots.org/projects/telekyb3>
- GenoM3: <https://www.openrobots.org/wiki/genom3>
- robotpkg: <http://robotpkg.openrobots.org/>
- Gazebo: <https://gazebosim.org/>
- SDFormat: <https://sdformat.org/>
- python-genomix: <https://git.openrobots.org/projects/genomix>

### GenoM3 components used here
Each link points at the up-to-date source/README. The version actually installed inside the `tk3lab`
Docker image may differ slightly; the offline doc shipped with the image at
`/opt/openrobots/share/doc/<component>-genom/README.html` is authoritative for the installed version.

- [`mrsim-gazebo`](https://git.openrobots.org/projects/mrsim-gazebo)
- [`optitrack-gazebo`](https://git.openrobots.org/projects/optitrack-gazebo)
- [`rotorcraft-genom3`](https://git.openrobots.org/projects/rotorcraft-genom3)
- [`pom-genom3`](https://git.openrobots.org/projects/pom-genom3)
- [`nhfc-genom3`](https://git.openrobots.org/projects/nhfc-genom3)
- [`uavpos-genom3`](https://git.openrobots.org/projects/uavpos-genom3)
- [`uavatt-genom3`](https://git.openrobots.org/projects/uavatt-genom3)
- [`maneuver-genom3`](https://git.openrobots.org/projects/maneuver-genom3)
- [`phynt-genom3`](https://git.openrobots.org/projects/phynt-genom3)
- [`optitrack-genom3`](https://git.openrobots.org/projects/optitrack-genom3)
