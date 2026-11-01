"""Run on a separate ROS_DOMAIN_ID; catches Node reserved-name and API errors."""
import unittest
import copy
import rclpy
from robotx_task1.mission import Mission
from robotx_task1.observer import KnownObserver
from robotx_task1_interfaces.msg import Buoy, BuoyArray


class RosStartup(unittest.TestCase):
    def test_stale_revision_and_semantic_cancel(self):
        rclpy.init();mission=Mission()
        try:
            msg=BuoyArray();msg.header.frame_id='map';msg.header.stamp=mission.get_clock().now().to_msg();msg.revision=1;msg.survey_complete=True
            for i in range(10):
                b=Buoy();b.id=str(i);b.position.x=float(i);b.collision_radius=.36;b.state=3 if i==0 else 4 if i==1 else 0
                msg.buoys.append(b)
            mission.on_observation(msg);self.assertEqual(mission.last_revision,1)
            newer=copy.deepcopy(msg);newer.header.stamp=mission.get_clock().now().to_msg();newer.revision=2;newer.buoys[2].state=1
            mission.started=True;mission.stage='TRANSIT';mission.motion='EXECUTING'
            mission.on_observation(newer)
            self.assertEqual(mission.motion,'STOPPING');self.assertEqual(mission.generation,1)
            mission.on_observation(msg);self.assertEqual(mission.last_revision,2);self.assertEqual(mission.generation,1)
            future=copy.deepcopy(newer);future.header.stamp.sec+=2;future.revision=3
            mission.on_observation(future);self.assertEqual(mission.last_revision,2)
        finally:mission.destroy_node();rclpy.try_shutdown()

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
