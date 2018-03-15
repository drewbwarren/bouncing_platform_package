#!/usr/bin/env python

import rospy
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

class dual_cam:

    def __init__(self):
        self.franco = PinholeCameraModel()
        self.laptop = PinholeCameraModel()

        self.sub = rospy.Subscriber("ball_position", Point, self.position_cb)
        self.sub2 = rospy.Subscriber("ball_position2", Point, self.position_cb2)

        self.pub = rospy.Publisher('/ray', Point, queue_size=10)
        
        
        msg = rospy.wait_for_message('/giacomo/camera_info', CameraInfo)
        self.franco.fromCameraInfo(msg)

        msg = rospy.wait_for_message('/laptop_cam/camera_info', CameraInfo)
        self.laptop.fromCameraInfo(msg)

        self.vec1 = []
        self.vec2 = []

    def position_cb(self, data):
        u = int(data.x)
        v = int(data.y)
        
        self.vec1 = self.laptop.projectPixelTo3dRay((u,v))
        rospy.loginfo(self.vec1)

    def position_cb2(self, data):
        u = int(data.x)
        v = int(data.y)
        
        self.vec2 = self.franco.projectPixelTo3dRay(((u,v)))
        


if __name__ == '__main__':
    rospy.init_node('dual_cam')

    dc = dual_cam()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown now")
