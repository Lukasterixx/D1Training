"""The task's controlled point, with no dependencies so the launcher can read it before Isaac starts.

The tip of the D1's Link7_1 pincer: the centre of its 26 x 7 mm end face, in Link7_1's own link
frame. From the CAD finger mesh (`workspace.pincer_tip`), not measured on the arm. It is attached to
the pincer rather than to Link6, so it stays exact as the jaw opens. At the arm's zero pose Link7_1
is the pincer on the robot's right, 12.5 cm ahead of the Link6 origin.
"""

TOOL_BODY = "Link7_1"
TOOL_OFFSET_M = (0.0547, 0.0060, 0.0170)
