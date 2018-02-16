import rospy from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ball_tracking:

    def __init__(self):
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError, e:
            print("---Camera Manage---", e)

        
        ######
        # Blob Tracker, color based
        ######

        # Color filter for orange ping pong ball
        lower = np.array([0,100,100])
        upper = np.array([9,255,255])

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(cv_image,cv_image, mask=mask) 

        erod = cv2.erode(mask, None, iterations=2)
        dilate = cv2.dilate(erode, None, iterations=2)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        x_max = 0
        y_max = 0
        x_cir = 0
        y_cir = 0
        r_max = 0
        for c in contours:
            (x_cir, y_cir), radius = cv2.minEnclosingCircle(c)
            if radius > r_max:
                r_max = radius
                x_max = x_cir
                y_max = y_cir

        # some filtering

        self.filterx[1:10] = self.filterx[0:9]
        self.filterx[0] = x_max
        if np.abs(x_max - self.x) > 10 :
            self.x = np.mean(self.filterx[0:5])
        else:
            self.x = np.mean(self.filterx[0:8])

        self.filtery[1:10] = self.filtery[0:9]
        self.filtery[0] = y_max
        if np.abs(y_max - self.y) > 10:
           self.y = np.mean(self.filtery[0:5])
        else:
            self.y = np.mean(self.filtery[0:8])


        cv2.circle(cv_image, (int(self.x), int(self.y)), 6, [255,255,255], -1)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


