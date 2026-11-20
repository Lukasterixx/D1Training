"""Where the base is on the dog while a whole-body policy runs, from its legs (item 1b, 2026-11-19). numpy only.

`press_live.py` has no odometry: sport mode's is gone once the legs are driven low level, and the foot press holds its
plate relative to the base. The learned insertion cannot: its task layer measures the key's depth from the arm's forward
kinematics placed by the base's pose against the lock, and in training that pose is the simulator's own. A body leaning
into the press with the base held fixed would read as the key backing out.

The estimate here is the textbook one for a robot standing on its feet: the feet in contact do not move. At `reset` each
foot in contact is anchored where the base's pose (given) and the leg's forward kinematics put it. Each `step` the base
is where the anchored feet put it -- foot minus the base's rotation (the IMU's) times the leg's kinematics, averaged
over the anchored feet in contact -- and a foot that comes down is anchored where it lands. With no foot anchored the
last estimate is held. The kinematics are the export's (`leg_policy.foot_local`, the URDF's numbers), in Unitree's leg
order, as the dog's `LowState` gives the joints.

What it assumes and does not see: feet that slip, the foot sphere rolling (its 22 mm radius, a few millimetres as the
calf pitches), the joint encoders' and the URDF's errors, and the IMU's yaw drift (small over a 10-20 s press). How well it
does against the simulator's base is `check_odometry.py`'s, on traces from `--insert_trace_body` runs.
"""
from __future__ import annotations

import numpy as np

from demos.combiner_key.robot import leg_policy

#: Unitree's leg order (`LowState`), and each leg's three joints' offset in its 12.
LEG_ORDER = ("FR", "FL", "RR", "RL")


def quat_matrix(quat_wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def feet_in_base(q_unitree, meta: dict) -> np.ndarray:
    """(4, 3): each foot sphere's centre in the base frame, Unitree's leg order."""
    q = np.asarray(q_unitree, dtype=float)
    return np.stack([leg_policy.foot_local(leg, q[3 * k:3 * k + 3], meta) for k, leg in enumerate(LEG_ORDER)])


class LegOdometry:
    """The base's position in a world frame from the feet in contact (Unitree's leg order throughout)."""

    def __init__(self, meta: dict):
        self.meta = meta
        self.anchor = np.full((4, 3), np.nan)
        self.base = np.zeros(3)
        self.used = 0

    def reset(self, base_pos_w, quat_wxyz, q_unitree, contact) -> None:
        """Anchor the feet in contact where `base_pos_w` and the legs put them."""
        self.base = np.asarray(base_pos_w, dtype=float).copy()
        world = self.base + feet_in_base(q_unitree, self.meta) @ quat_matrix(quat_wxyz).T
        self.anchor = np.where(np.asarray(contact, dtype=bool)[:, None], world, np.nan)
        self.used = int(np.asarray(contact, dtype=bool).sum())

    def step(self, quat_wxyz, q_unitree, contact) -> np.ndarray:
        """The base's position now; anchors the feet that have just come down."""
        contact = np.asarray(contact, dtype=bool)
        rot = quat_matrix(quat_wxyz)
        arm = feet_in_base(q_unitree, self.meta) @ rot.T           # base to each foot, in the world's axes
        planted = contact & ~np.isnan(self.anchor[:, 0])
        self.used = int(planted.sum())
        if self.used:
            self.base = (self.anchor[planted] - arm[planted]).mean(axis=0)
        # A foot that is up loses its anchor; one that has come down is anchored where it is now.
        self.anchor[~contact] = np.nan
        landed = contact & np.isnan(self.anchor[:, 0])
        self.anchor[landed] = self.base + arm[landed]
        return self.base.copy()
