"""Run on a separate ROS_DOMAIN_ID; catches Node reserved-name and API errors."""
import unittest
import rclpy
from robotx_task1.mission import Mission
from robotx_task1.observer import KnownObserver


class RosStartup(unittest.TestCase):
    def test_nodes_start_with_closed_permission(self):
        rclpy.init();mission=None;observer=None
        try:
            mission=Mission();observer=KnownObserver()
            self.assertFalse(mission.started);self.assertEqual(mission.stage,'WAIT_SURVEY')
            self.assertFalse(mission.inputs_valid())
            mission.tick()  # Includes actual generated context and permission messages.
        finally:
            if observer:observer.destroy_node()
            if mission:mission.destroy_node()
            rclpy.try_shutdown()


if __name__=='__main__':unittest.main()
