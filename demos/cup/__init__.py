"""The cup pick: a lying Go2 finds a cup with a wrist camera and picks it up, in simulation and on the arm.

* `demos/cup/pick_demo/` -- the scene, the wrist RealSense model, YOLO-plus-depth perception, the grasp planner and
  the state machine that drives both the simulator and the real D1.
* `demos/cup/run_pick_demo.py` -- the simulated pick (`demos/cup/run_pick_demo.sh` sets the Isaac environment up for it).
* `demos/cup/d1_ui/` -- the reach console: the arm's teleoperation page, the wrist mount editor, the gripper jog and
  the button that runs the pick on the real arm. It also serves the simulator's joint and camera feed,
  which the walking playback in `sim.py` publishes to.
* `demos/cup/run_ui.sh` -- launches that console here or deploys it to the dog.
* `run_camera_body_view.py` -- photographs the wrist camera's own housing, for checking the mount.
* `tests/` -- everything above, on the CPU model, with no simulator and no arm.
"""
