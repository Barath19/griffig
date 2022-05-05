#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from griffig import Affine, Griffig, Gripper, Pointcloud, BoxData


box_data = BoxData(
    center=(0.0, 0.017, 0.0),  # At the center [m]
    size=(0.18, 0.285, 0.1),  # (x, y, z) [m]
)

gripper = Gripper(  # Some information about the gripper
    min_stroke=0.01,  # Min. pre-shaped width in [m]
    max_stroke=0.10,  # Max. pre-shaped width in [m]
)

griffig = Griffig(
    model='two-finger-planar',  # Use the default model for a two-finger gripper
    gripper=gripper,
    box_data=box_data,
    typical_camera_distance=0.41,  # Typical distance between camera and bin (used for depth image rendering) [m]
)

    # Return image (depth channel) with grasp for visualization
    
class SubscribePointCloud(object):
    def __init__(self):
        rospy.init_node('subscribe_custom_point_cloud')
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)
        rospy.spin()

    def callback(self, point_cloud):
        pointcloud = Pointcloud(ros_message=point_cloud)
        print(pointcloud)
        #grasp, image = griffig.calculate_grasp(pointcloud, return_image=True, channels='D')
        #image = griffig.render(pointcloud)
        #print(grasp)
        #image.show()
        #for point in pc2.read_points(point_cloud):
            #rospy.logwarn("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))
            #rospy.logwarn("my field 1: %f" % (point[4]))
            #rospy.logwarn("my field 2: %f" % (point[5]))


def main():
    try:
        SubscribePointCloud()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()