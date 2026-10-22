"""The legs' motor models for the front-right-leg task (`--leg_actuator`), a diagnostic of what the policies trained on.

The leg and key policies trained on UniFP's ideal PD (`unifp`): the URDF's torque limits and nothing else. The walking
policies this project deploys (rima's, P2Dingo's) trained on Isaac Lab's stock Go2 `DCMotor`, which gives the knee the
hip's 23.5 N·m (F-165: the force-aware key policy 84 -> 3 of 108 on it). These models separate the two differences and add
Unitree's own envelope, with and without the geared knee the URDF implies (`motor_model.GO2_HV_KNEE`, derived, not
measured). Every model keeps the 25 / 0.5 gains (joint-side, as the Go2's LowCmd takes them and rima's deploy node sends
them) and the trained legs' armature; only the torque law changes, plus unitree_rl_lab's joint friction (0.01) on its two.

| name             | hip, thigh (N·m) | knee (N·m)   | with speed                                                      |
| ---------------- | ---------------- | ------------ | --------------------------------------------------------------- |
| `unifp`          | 23.7             | 45.43        | none (trained on)                                               |
| `dcmotor`        | 23.5             | 23.5         | linear to 0 at 30 rad/s (Isaac Lab's `UNITREE_GO2_CFG`)          |
| `knee23`         | 23.7             | 23.5         | none: `unifp` with only the knee's cap changed                   |
| `speedcurve`     | 23.7             | 45.43        | linear to 0 at 30 rad/s: `unifp` with only DCMotor's speed curve |
| `knee23curve`    | 23.7             | 23.5         | linear to 0 at 30 rad/s: both, `dcmotor` with the trained armature |
| `dcmotor_armature` | 23.5           | 23.5         | `dcmotor` with only the legs' armature set to the trained 2e-4    |
| `unitree`        | 20.2 / 23.4      | 20.2 / 23.4  | flat to 13.5 rad/s, 0 at 30 (unitree_rl_lab's GO2HV, F-005)       |
| `unitree_geared` | 20.2 / 23.4      | 38.7 / 44.9  | hip, thigh as `unitree`; knee flat to 7.0 rad/s, 0 at 15.65      |

(a / b: driving / braking torque.) `armature` (and `knee_armature`, default the same) replaces the legs' armature in any
model, kg·m²: the policies trained with UniFP's 2e-4; unitree_rl_lab gives the same GO2HV motor 0.01 where it drives the
H1's joints (it leaves the Go2's at the robot file's 0); a knee geared 1.917 more reflects 1.917² as much. `delay_steps` adds a command delay of 0 to that many physics steps (5 ms each), drawn
per environment, to the models whose Isaac Lab actuator has a delay buffer.
"""
from __future__ import annotations

import copy

import motor_model
from unifp_isaaclab import interface

LEG_ACTUATORS = ("unifp", "dcmotor", "knee23", "speedcurve", "knee23curve", "dcmotor_armature", "unitree",
                 "unitree_geared")
DELAYABLE = ("unifp", "knee23", "unitree", "unitree_geared")

#: Isaac Lab's stock Go2 `DCMotor` (`isaaclab_assets.robots.unitree.UNITREE_GO2_CFG`).
STOCK_NM, STOCK_RAD_S = 23.5, 30.0
#: unitree_rl_lab's Go2 joint friction.
UNITREE_FRICTION = 0.01

LEGS = list(interface.ISAACLAB_NAMES[:12])
HIP_THIGH = [n for n in LEGS if not n.endswith("calf_joint")]
KNEES = [n for n in LEGS if n.endswith("calf_joint")]
REST = list(interface.ISAACLAB_NAMES[12:])


def _only(value, names):
    return {k: v for k, v in value.items() if k in names} if isinstance(value, dict) else value


def describe(name: str, delay_steps: int = 0, armature: float | None = None,
             knee_armature: float | None = None, min_delay_steps: int = 0) -> dict:
    """The model's limits, for the run record (N·m, rad/s)."""
    hv, knee = motor_model.GO2_HV, motor_model.GO2_HV_KNEE
    ideal = lambda ht, k: {"hip_thigh_nm": ht, "knee_nm": k, "speed": "none"}
    table = {
        "unifp": ideal(23.7, 45.43),
        "knee23": ideal(23.7, STOCK_NM),
        "dcmotor": {"hip_thigh_nm": STOCK_NM, "knee_nm": STOCK_NM, "speed": f"linear to 0 at {STOCK_RAD_S} rad/s"},
        "speedcurve": {"hip_thigh_nm": 23.7, "knee_nm": 45.43, "speed": f"linear to 0 at {STOCK_RAD_S} rad/s"},
        "knee23curve": {"hip_thigh_nm": 23.7, "knee_nm": STOCK_NM, "speed": f"linear to 0 at {STOCK_RAD_S} rad/s"},
        "dcmotor_armature": {"hip_thigh_nm": STOCK_NM, "knee_nm": STOCK_NM, "speed": f"linear to 0 at {STOCK_RAD_S} rad/s",
                             "armature": "the trained legs'"},
        "unitree": {"hip_thigh_nm": [hv.Y1, hv.Y2], "knee_nm": [hv.Y1, hv.Y2],
                    "speed": f"flat to {hv.X1}, 0 at {hv.X2} rad/s", "friction": UNITREE_FRICTION},
        "unitree_geared": {"hip_thigh_nm": [hv.Y1, hv.Y2], "knee_nm": [round(knee.Y1, 2), round(knee.Y2, 2)],
                           "speed": f"hip, thigh flat to {hv.X1}, 0 at {hv.X2}; knee flat to {knee.X1:.2f}, "
                                    f"0 at {knee.X2:.2f} rad/s", "knee_ratio": round(motor_model.GO2_KNEE_RATIO, 4),
                           "friction": UNITREE_FRICTION},
    }
    out = {"name": name, **table[name], "gains": [25.0, 0.5], "delay_physics_steps": [int(min_delay_steps), int(delay_steps)]}
    if armature is not None:
        out["armature_kg_m2"] = {"hip_thigh": armature, "knee": armature if knee_armature is None else knee_armature}
    return out


def swap_legs(robot_cfg, name: str, delay_steps: int = 0, armature: float | None = None,
              knee_armature: float | None = None, min_delay_steps: int = 0):
    """Give the legs of `robot_cfg` (`unifp_isaaclab.robot.make_robot_cfg`'s, one 'unifp' group) the model `name`.

    The arm and the jaws keep the 'unifp' group as it was. In place; returns `robot_cfg`. Import only after Isaac Sim has
    started.
    """
    if name not in LEG_ACTUATORS:
        raise ValueError(f"Unknown leg actuator {name!r}; one of {LEG_ACTUATORS}")
    if delay_steps and name not in DELAYABLE:
        raise ValueError(f"Leg actuator {name!r} has no delay buffer; delay works with {DELAYABLE}")
    if name == "unifp" and not delay_steps and armature is None:
        return robot_cfg
    from isaaclab.actuators import DCMotorCfg, DelayedPDActuatorCfg, IdealPDActuatorCfg

    from unitree_actuators import UnitreeActuatorCfg, UnitreeGo2HVActuatorCfg

    group = robot_cfg.actuators["unifp"]

    def part(names, **extra):
        """The trained group's own values for `names` (gains, armature, friction), with `extra` over them."""
        base = dict(joint_names_expr=list(names), stiffness=_only(group.stiffness, names),
                    damping=_only(group.damping, names), armature=_only(group.armature, names),
                    friction=_only(group.friction, names))
        return base | extra

    arm = group.replace(**part(REST, effort_limit=_only(group.effort_limit, REST),
                               effort_limit_sim=_only(group.effort_limit_sim, REST)))
    if min_delay_steps > delay_steps:
        raise ValueError(f"min_delay_steps {min_delay_steps} is past delay_steps {delay_steps}")
    delay = dict(min_delay=int(min_delay_steps), max_delay=int(delay_steps))
    if name in ("unifp", "knee23"):
        effort = _only(group.effort_limit, LEGS)
        if name == "knee23":
            effort = {k: (STOCK_NM if k in KNEES else v) for k, v in effort.items()}
        legs = {"legs": DelayedPDActuatorCfg(**part(LEGS, effort_limit=effort, effort_limit_sim=effort), **delay)}
    elif name in ("dcmotor", "dcmotor_armature"):
        # Yesterday's swap, unchanged (F-165), so its result reproduces: the stock group, which sets no armature (the
        # robot file's stays) and friction 0. `dcmotor_armature` gives it the trained legs' armature and nothing else.
        from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

        stock = copy.deepcopy(UNITREE_GO2_CFG.actuators["base_legs"])
        legs = {"base_legs": stock if name == "dcmotor" else stock.replace(armature=_only(group.armature, LEGS))}
    elif name in ("speedcurve", "knee23curve"):
        knee_nm = 45.43 if name == "speedcurve" else STOCK_NM
        legs = {"legs_hip_thigh": DCMotorCfg(**part(HIP_THIGH, saturation_effort=23.7, effort_limit=23.7,
                                                    velocity_limit=STOCK_RAD_S)),
                "legs_knee": DCMotorCfg(**part(KNEES, saturation_effort=knee_nm, effort_limit=knee_nm,
                                               velocity_limit=STOCK_RAD_S))}
    elif name == "unitree":
        legs = {"legs": UnitreeGo2HVActuatorCfg(**part(LEGS, friction=UNITREE_FRICTION), **delay)}
    else:
        knee = motor_model.GO2_HV_KNEE
        legs = {"legs_hip_thigh": UnitreeGo2HVActuatorCfg(**part(HIP_THIGH, friction=UNITREE_FRICTION), **delay),
                "legs_knee": UnitreeActuatorCfg(**part(KNEES, friction=UNITREE_FRICTION), X1=knee.X1, X2=knee.X2,
                                                Y1=knee.Y1, Y2=knee.Y2, **delay)}
    if armature is not None:
        knee = armature if knee_armature is None else knee_armature
        for group_cfg in legs.values():
            names = HIP_THIGH + KNEES if group_cfg.joint_names_expr == [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"] \
                else group_cfg.joint_names_expr
            group_cfg.armature = {n: (knee if n in KNEES else armature) for n in names}
    robot_cfg.actuators = {**legs, "unifp": arm}
    return robot_cfg
