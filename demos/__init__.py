"""Demonstrations built on this repository's robot: one package per demo.

A demo is a task the Go2 and its welded D1 can be watched doing end to end -- scene, perception,
planning, the scripted sequence and whatever console drives it -- kept together so that reading one does
not mean reading the whole repository. What stays outside is what every demo shares: the arm's kinematics
(`d1_ik`), its DDS client (`d1_hardware`), the walking policy and the position-only task.
"""
