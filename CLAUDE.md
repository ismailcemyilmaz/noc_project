# CLAUDE.md — NOC project (TiltHex variable impedance control)

## Project
- Goal: variable impedance control for the fully-actuated TiltHex in contact with a wall.
- Optimization objective is NOT fixed yet. Current phase: measurement + infrastructure.
- Base: telekyb3 lab repo, reduced to physical interaction: `src/06a-physical-interaction/`,
  `gazebo/models/mrsim-tilthex`, `gazebo/worlds/hexa-fa-wall-world.world`. Lab assignment 06a is the baseline.
  Labs 01–05b were removed from the tree; read them with `git show b39a541:<path>`.

## Paths
- Host: `~/noc-ws`  ↔  container: `/shared-workspace` (same directory, bind-mounted).
- New code goes under `noc/` (`noc/common/params.py` = single source of physical parameters).
- Logs: `logs/` (gitignored). Results/plots: `noc/results/<experiment>/`.

## Rules
- Simulation runs inside Docker. Do NOT start the simulation yourself: give the user exact
  container commands (T1/T2), then analyse the logs afterwards.
- Never touch `~/tk3lab-ws`. Never `git push`. Make one meaningful commit per finished task.
- Every experiment must be reproducible from: config file + raw logs + seed.
- Never report a computed value as a measurement. State assumptions separately.
- Do not modify `src/06a-physical-interaction/`. Flight experiments load it through
  `noc/common/setup_06a.py` and use these defaults from `noc/common/flight.py`:
  - `setup_06a.setup(m)`: 06a setup() plus `uavatt.rotor_measure <- rotorcraft/rotor_measure`.
  - `flight.goto(m, args)`: `ack=True`, non-blocking.
  - `flight.Safety`.
  Redirect `LOG_DIR` to `logs/noc/<exp>/`; 06a `stop()` otherwise overwrites `logs/06a-*`.
- Reference controller/AF config: `noc/config/af_on_baseline.json` (AF-on, run t01).
- genomix from Python: read only output ports and attribute services. Input ports such as
  `phynt.reference` need a path argument; use `noc/common/telekyb_io.py`.

## Known inconsistencies
- Mass: 05b (`git show b39a541:src/05b-motion-control/model_hexa_fa.py`) uses 2.3 kg (base only); 06a uses 2.72 kg (base + 6 × 0.07 kg rotors).
- Tip radius: SDF 0.04 m; plot_06a.py used 0.05 m (now fixed to 0.04).
- plot_06a.py does NOT measure contact force: it computes K_WALL × penetration (idealised spring).
- Tip centre in SDF is at body `[0.6, 0, -0.125]`; plot_06a.py EE kinematics ignore the z offset.
- Rotor tilt: SDF rotor poses ≈ 20°; uavatt/compute_G use rx=-21.2°, ry=-18.7° (unverified mapping).
- Inertia is base-link only (rotor inertia ignored).
- 06a setup() never connects uavatt `rotor_measure`. Without the fix, `wrench_measure.force` is None,
  the WO and AF are inactive and phynt.log stays empty (`noc/results/00b_diagnosis.md`).
- 06a `maneuver.goto` without `ack` blocks for the whole motion (2–5 s).
- When idle, `maneuver.desired` has all fields None, and the phynt reference has pos None with vel 0.
- Neither the WO `external_wrench` nor uavpos `fx` is a contact-force measurement; both are estimates.

## Tests
- `.venv/bin/python -m unittest discover -s noc/tests -t .` (host, no Docker needed).
