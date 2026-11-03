# CLAUDE.md — NOC project (TiltHex variable impedance control)

## Project
- Goal: variable impedance control for the fully-actuated TiltHex in contact with a wall.
- Optimization objective is NOT fixed yet. Current phase: measurement + infrastructure.
- Base: telekyb3 lab repo (`src/`, `gazebo/`). Lab assignment 06a is the baseline.

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

## Known inconsistencies
- Mass: 05b uses 2.3 kg (base only); 06a uses 2.72 kg (base + 6 × 0.07 kg rotors).
- Tip radius: SDF 0.04 m; plot_06a.py used 0.05 m (now fixed to 0.04).
- plot_06a.py does NOT measure contact force: it computes K_WALL × penetration (idealised spring).
- Tip centre in SDF is at body `[0.6, 0, -0.125]`; plot_06a.py EE kinematics ignore the z offset.
- Rotor tilt: SDF rotor poses ≈ 20°; uavatt/compute_G use rx=-21.2°, ry=-18.7° (unverified mapping).
- Inertia is base-link only (rotor inertia ignored).

## Tests
- `.venv/bin/python -m unittest discover -s noc/tests -t .` (host, no Docker needed).
