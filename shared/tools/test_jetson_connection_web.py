import subprocess
import unittest
from unittest.mock import patch

from jetson_connection_web import Controller


class JetsonConnectionRecoveryTest(unittest.TestCase):
    def controller(self):
        controller = object.__new__(Controller)
        controller.logs = []
        controller.lock = __import__("threading").Lock()
        return controller

    @patch("jetson_connection_web.time.sleep")
    @patch("jetson_connection_web.subprocess.run")
    def test_resets_expected_l4t_profile(self, run, sleep):
        run.side_effect = [
            subprocess.CompletedProcess(
                [],
                0,
                stdout=(
                    "Manual Configuration\n"
                    "IP address: 192.168.55.100\n"
                ),
                stderr="",
            ),
            subprocess.CompletedProcess([], 0, stdout="", stderr=""),
            subprocess.CompletedProcess([], 0, stdout="", stderr=""),
        ]
        recovered = self.controller().recover_l4t_usb_network(
            [("192.168.55.1", 22), ("192.168.2.2", 22)]
        )
        self.assertTrue(recovered)
        self.assertEqual(run.call_count, 3)
        sleep.assert_called_once_with(1.0)

    @patch("jetson_connection_web.subprocess.run")
    def test_skips_unknown_profile(self, run):
        run.return_value = subprocess.CompletedProcess(
            [],
            0,
            stdout="DHCP Configuration\n",
            stderr="",
        )
        recovered = self.controller().recover_l4t_usb_network(
            [("192.168.55.1", 22)]
        )
        self.assertFalse(recovered)
        self.assertEqual(run.call_count, 1)

    @patch("jetson_connection_web.subprocess.run")
    def test_skips_when_usb_endpoint_not_requested(self, run):
        recovered = self.controller().recover_l4t_usb_network(
            [("192.168.2.2", 22)]
        )
        self.assertFalse(recovered)
        run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
