#!/usr/bin/env python
#***************************************************************************
#
# Copyright (c) 2015 UAVenture AG. All rights reserved.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
PKG = 'vertical_takeoff'

import unittest
import rospy
import math
import rosbag

from numpy import linalg
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from px4_test_helper import PX4TestHelper

class VerticalTakeoffTest(unittest.TestCase):
    """
    Tests a vertical takeoff.
    """

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        self.helper = PX4TestHelper("vertical_takeoff_test")
        self.helper.setUp()

        rospy.Subscriber("iris/mavros/position/local", PoseStamped, self.position_callback)
        self.pub_spt = rospy.Publisher('iris/mavros/setpoint/local_position', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.has_pos = False
        self.local_position = PoseStamped()

    def tearDown(self):
        self.helper.tearDown()

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.has_pos = True
        self.local_position = data


    #
    # Helper methods
    #
    def is_at_position(self, x, y, z, offset):
        if not self.has_pos:
            return False

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z

        # set a default heading
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in X seconds?
        count = 0
        while count < timeout:
            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_spt.publish(pos)
            self.helper.bag_write('mavros/setpoint/local_position', pos)

            if self.is_at_position(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, 0.5):
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to get to position")

    #
    # Test method
    #
    def test_vertical_takeoff(self):
        """Command a vertical takeoff to 10m above gorund"""

        self.reach_position(0, 0, 10, 100)



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'vertical_takeoff_test', VerticalTakeoffTest)
    #unittest.main()
