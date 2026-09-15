"""CPU checks for the D1 kinematics and mass model in position_only/workspace.py."""
import unittest

import numpy as np

from position_only import workspace


class WorkspaceModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joints, cls.links = workspace.load_urdf()

    def test_mass_model_totals_the_published_arm_mass(self):
        masses = workspace.simulated_masses(self.links)
        self.assertAlmostEqual(sum(masses.values()), workspace.ARM_MASS_KG, places=9)
        moving = sum(m for name, m in masses.items() if name != "base_link")
        self.assertAlmostEqual(moving, 0.987, places=3)  # weld.py: "~1.0 kg of moving arm"

    def test_zero_pose_puts_link6_forward_at_the_target_box_height(self):
        # Pinned to the value the simulator reproduced to 0.6 um in the Week 1 verify run.
        frames, _, _ = workspace.forward(self.joints, np.zeros((1, 6)))
        np.testing.assert_allclose(frames["Link6"][1][0], [0.2804, -0.0013, 0.4999], atol=1e-4)

    def test_gravity_torque_is_the_gradient_of_potential_energy(self):
        q = np.random.default_rng(3).uniform(-1.0, 1.0, size=(4, 6))
        masses = workspace.simulated_masses(self.links)

        def potential(angles):
            frames, _, _ = workspace.forward(self.joints, angles)
            energy = np.zeros(len(angles))
            for name, (rot, pos) in frames.items():
                if name != "base_link":
                    com = pos + np.einsum("nij,j->ni", rot, self.links[name]["com"])
                    energy += masses[name] * -workspace.GRAVITY[2] * com[:, 2]
            return energy

        step, gradient = 1e-6, np.zeros_like(q)
        for j in range(6):
            offset = np.zeros(6)
            offset[j] = step
            gradient[:, j] = (potential(q + offset) - potential(q - offset)) / (2 * step)
        np.testing.assert_allclose(workspace.gravity_torques(self.joints, self.links, q), gradient, atol=1e-6)

    def test_task_tool_point_is_the_pincer_end_face_centre(self):
        from position_only.tool_point import TOOL_BODY, TOOL_OFFSET_M

        np.testing.assert_allclose(workspace.pincer_tip(TOOL_BODY), TOOL_OFFSET_M, atol=6e-5)
        frames, _, _ = workspace.forward(self.joints, np.zeros((1, 6)))
        np.testing.assert_allclose(workspace.tool_position(frames)[0], [0.4057, -0.0139, 0.4995], atol=1e-4)

    def test_opening_the_jaw_slides_each_pincer_outwards_along_link6_y(self):
        q = np.zeros((1, 6))
        closed, _, _ = workspace.forward(self.joints, q)
        opened, _, _ = workspace.forward(self.joints, q, gripper=np.array([[0.02, -0.02]]))
        # At the zero pose Link6's y axis is the base's y axis: Link7_1 moves to -y, Link7_2 to +y.
        np.testing.assert_allclose(opened["Link7_1"][1] - closed["Link7_1"][1], [[0.0, -0.02, 0.0]], atol=1e-4)
        np.testing.assert_allclose(opened["Link7_2"][1] - closed["Link7_2"][1], [[0.0, 0.02, 0.0]], atol=1e-4)

    def test_fingertips_sit_12_5_cm_along_link6_z(self):
        tips = workspace.tip_offsets(self.joints)
        self.assertAlmostEqual(tips["fingertip_centre_m"][2], 0.125, delta=0.002)
        self.assertLess(abs(tips["fingertip_centre_m"][0]) + abs(tips["fingertip_centre_m"][1]), 0.002)


if __name__ == "__main__":
    unittest.main()
