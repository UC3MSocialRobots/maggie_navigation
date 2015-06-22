#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Raul Perula-Martinez"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Raul Perula-Martinez"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Raul Perula-Martinez"
__email__ = "raul.perula@uc3m.es"
__status__ = "Development"

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

import roslib
import rosparam
import rospy


pkg_name = "maggie_navigation_config"
roslib.load_manifest(pkg_name)


class InitialposeSender():

    """
    InitialposeSender class.
    """

    def __init__(self):
        """
        Init method.
        """

        # publishers and subscribers
        self.__initialpose_pub = rospy.Publisher('initialpose',
                                                 PoseWithCovarianceStamped, queue_size=1, latch=True)

    def run(self):
        """
        Spinner of the node.

        @raise KeyError: exception when dictionary has an invalid key.
        @raise rosparam.RosParamException: exception when cannot open the config file.
        """

        try:
            # read the data from the yaml file
            data = rosparam.load_file(roslib.packages.get_pkg_dir(pkg_name) + "/config/initial_pose.yaml",
                                      rospy.get_name())
            try:
                # the place label come from the request
                place_data = data[0][0]

                # fill the msg with the data of the file
                msg = PoseWithCovarianceStamped()

                msg.header.frame_id = "/map"
                msg.header.stamp = rospy.Time.now()

                msg.pose.pose.position.x = place_data['x']
                msg.pose.pose.position.y = place_data['y']
                msg.pose.pose.position.z = 0

                msg.pose.covariance[0] = place_data['cov_x']
                msg.pose.covariance[7] = place_data['cov_y']
                msg.pose.covariance[35] = place_data['cov_th']

                # roll, pitch, yaw
                quat = quaternion_from_euler(0.0, 0.0, place_data['th'])
                msg.pose.pose.orientation = Quaternion(*quat.tolist())
                print msg.pose.pose.orientation.z
                print msg.pose.pose.orientation.w

#                 msg.pose.pose.orientation.x = 0
#                 msg.pose.pose.orientation.y = 0
#                 print place_data['th']
#                 msg.pose.pose.orientation.z = math.sin(
#                     (place_data['th'] - math.pi / 2.0) / 2.0)
#                 print msg.pose.pose.orientation.z
#                 msg.pose.pose.orientation.w = math.cos(
#                     (place_data['th'] - math.pi / 2.0) / 2.0)
#                 print msg.pose.pose.orientation.w

                # publish the initial position
                self.__initialpose_pub.publish(msg)
            except KeyError:
                rospy.logerr('[INITIAL_STATUS_SENDER] Error.')
        except rosparam.RosParamException:
            rospy.logerr('[INITIAL_STATUS_SENDER] Error loading config file')


if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(pkg_name)

        # create and spin the node
        node = InitialposeSender()
        node.run()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
