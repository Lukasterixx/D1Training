"""Scripted cup pick with a wrist RealSense and YOLO: no learning, the existing IK and the measured arm.

The pieces split the same way `d1_ik` does. `camera`, `perception` (except the YOLO wrapper), `grasp`
and `sequence` are numpy only, so they run in tests on the system Python and could drive the real arm;
`cup_asset` and `scene` need Isaac, and `run_pick_demo.py` puts them together in the simulator.
"""
