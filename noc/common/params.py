"""
Single source of physical parameters for the TiltHex (mrsim-tilthex) + wall setup.

Every value carries its source. "SDF" = gazebo/models/mrsim-tilthex/model.sdf,
"WORLD" = gazebo/worlds/hexa-fa-wall-world.world, "06a" = src/06a-physical-interaction/model_hexa_fa.py,
"05b" = src/05b-motion-control/model_hexa_fa.py.

Values marked ASSUMPTION are not read directly from a simulator file.
"""

import numpy as np

# ---------------------------------------------------------------------------
#  Gravity
# ---------------------------------------------------------------------------
G_ACC = 9.81  # WORLD <gravity>0 0 -9.81</gravity>

# ---------------------------------------------------------------------------
#  Mass / inertia
# ---------------------------------------------------------------------------
MASS_BASE = 2.3          # SDF base link <mass>; also 05b MASS
MASS_ROTOR = 0.07        # mrsim-rotor model.sdf <mass> (copy outside this repo; README 6a)
N_ROTORS = 6             # SDF: rotor-1..rotor-6
MASS = MASS_BASE + N_ROTORS * MASS_ROTOR  # = 2.72 kg, 06a MASS (Gazebo total)

# SDF base link inertia. ASSUMPTION: rotor/end-effector inertia contributions ignored
# (same as 05b/06a, which pass these numbers to the controllers).
J = np.diag([0.0115, 0.0114, 0.0194])

# ---------------------------------------------------------------------------
#  Rotors
# ---------------------------------------------------------------------------
ARMLEN = 0.38998         # SDF rotor-1 pose x; 05b/06a ARMLEN
CF = 9.9016e-4           # SDF mrsim plugin <cf>  (thrust = CF * w^2)
CT = 1.9e-5              # SDF mrsim plugin <ct>  (drag torque = CT * w^2)
RZ = -1                  # 05b/06a uavatt set_gtmrp_geom 'rz' (spin direction convention)
# Tilt angles as given to uavatt set_gtmrp_geom (06a) and compute_G (05b).
# NOTE: SDF rotor poses encode ~20 deg (0.3491 rad); mapping to rx/ry not verified.
RX_DEG = -21.2
RY_DEG = -18.7
# Rotor speed limits [Hz-like units used by uavatt set_wlimit] — 06a set_wlimit, 05b OMEGA_*
OMEGA_MIN = 16.0
OMEGA_MAX = 100.0

# ---------------------------------------------------------------------------
#  End-effector geometry (body frame)
# ---------------------------------------------------------------------------
L_BAR = 0.6                                   # SDF ee-hbar length 0.6 (x from 0 to 0.6)
EE_BAR_Z = -0.125                             # SDF ee-hbar / ee-tip pose z
TIP_RADIUS = 0.04                             # SDF ee-tip-col sphere radius
TIP_CENTER_B = np.array([L_BAR, 0.0, EE_BAR_Z])  # SDF ee-tip pose
TIP_MU = 0.3                                  # SDF ee-tip-col friction mu/mu2

# ---------------------------------------------------------------------------
#  Wall (WORLD contact_wall)
# ---------------------------------------------------------------------------
WALL_CENTER_X = 2.01     # WORLD contact_wall pose x
WALL_THICKNESS = 0.02    # WORLD box size x
X_WALL = WALL_CENTER_X - WALL_THICKNESS / 2  # = 2.00, front face; 06a X_WALL
WALL_SIZE_YZ = (4.0, 4.0)                    # WORLD box size y, z
WALL_CENTER_Z = 2.0                          # WORLD contact_wall pose z
K_WALL = 500.0           # WORLD <kp> (ODE contact stiffness; ASSUMPTION: Gazebo honours it linearly)
D_WALL = 50.0            # WORLD <kd>
WALL_MU = 0.3            # WORLD friction mu/mu2
WALL_MIN_DEPTH = 0.001   # WORLD <min_depth>


# ---------------------------------------------------------------------------
#  Allocation matrix (moved from 05b)
# ---------------------------------------------------------------------------
def compute_G(n_rotors=N_ROTORS, armlen=ARMLEN, cf=CF, ct=CT, rz=RZ,
              rx_deg=RX_DEG, ry_deg=RY_DEG):
    """6 x n allocation matrix: wrench_body = G @ (w * |w|). Copied verbatim from 05b."""
    rx = np.radians(rx_deg)
    ry = np.radians(ry_deg)
    G = np.zeros((6, n_rotors))
    sign = 1
    for i in range(n_rotors):
        theta = 2 * np.pi * i / n_rotors

        Rz = np.array([[np.cos(theta), -np.sin(theta), 0],
                       [np.sin(theta),  np.cos(theta), 0],
                       [0,              0,             1]])
        srx = sign * rx
        Rx = np.array([[1, 0,            0           ],
                       [0, np.cos(srx), -np.sin(srx)],
                       [0, np.sin(srx),  np.cos(srx)]])
        Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                       [ 0,          1, 0         ],
                       [-np.sin(ry), 0, np.cos(ry)]])

        z = (Rz @ Rx @ Ry)[:, 2]
        p = armlen * np.array([np.cos(theta), np.sin(theta), 0.0])

        G[0:3, i] = cf * z
        G[3:6, i] = cf * np.cross(p, z) - sign * rz * ct * z
        sign = -sign
    return G
