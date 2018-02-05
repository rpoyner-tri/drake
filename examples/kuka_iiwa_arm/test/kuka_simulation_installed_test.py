
import os
import shutil
import subprocess
import unittest
import sys
import install_test_helper


class TestKukaSimulation(unittest.TestCase):
    def test(self):
        # Install into the tmpdir that Bazel has created for us.
        install_dir = install_test_helper.install()

        # Make sure the simulation can run without error.  We set cwd="/" to
        # defeat the "search in parent folders" heuristic, so that the "use
        # libdrake.so relative paths" must be successful.
        simulation = os.path.join(
            install_dir,
            "share/drake/examples/kuka_iiwa_arm/kuka_simulation")
        self.assertTrue(os.path.exists(simulation), "Can't find " + simulation)
        subprocess.check_call([simulation, "--simulation_sec=0.01"], cwd="/")


if __name__ == '__main__':
    unittest.main()
