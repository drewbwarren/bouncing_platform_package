#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point
from stewart_platform.msg import Reference_Pos

class ball_tracking:

    def __init__(self):
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.filterx = [0,0,0,0,0,0,0,0,0,0]
        self.filtery = [0,0,0,0,0,0,0,0,0,0]
        self.x = 0
        self.y = 0

        self.corners = [[0,0],[0,0],[0,0],[0,0]]
        self.corner_filter = [[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]], \
            [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]], \
            [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]], \
            [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]]

        self.img_size = 300
        self.brd_size = .25
        self.pub = rospy.Publisher('/ball_position', Point, queue_size=1)
        self.ref_sub = rospy.Subscriber('reference_position', Reference_Pos, self.ref_cb)
        self.refx = 0
        self.refy = 0

    def ref_cb(self, data):
        self.refx = data.x
        self.refy = data.y
        
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError, e:
            print("---Camera Manage---", e)


        platform_img = self.perspective_image(cv_image)
        self.ball_locator(platform_img)
        newpt = Point()
        newpt.x = ((self.x + self.img_size/2) / self.img_size) * self.brd_size - self.brd_size
        newpt.y = -((self.y + self.img_size/2) / self.img_size) * self.brd_size + self.brd_size
        newpt.z = 0
        self.pub.publish(newpt)



    def ball_locator(self, platform_img):
        ######
        # Blob Tracker, color based
        ######

        # Color filter for orange ping pong ball
        lower = np.array([10,200,130])
        upper = np.array([100,255,255]) # 65 for darker lighting
        upper = np.array([30,255,255])

        # Color filter for white ball
        #lower = np.array([50,0,int(70*255/100.0)])
        #upper = np.array([200,int(15*255/100.0),255])


        hsv = cv2.cvtColor(platform_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
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


        self.x = x_max
        self.y = y_max

        refx_img = (self.refx + self.brd_size)*self.img_size/self.brd_size - self.img_size/2
        refy_img = -(self.refy - self.brd_size)*self.img_size/self.brd_size - self.img_size/2

        cv2.circle(platform_img, (int(self.x), int(self.y)), 5, [255,255,255], -1)
        cv2.circle(platform_img, (int(refx_img),int(refy_img)), 4, [255,0,0],-1)

        cv2.imshow("Image window", platform_img)
        cv2.imshow("color separate",mask)
        cv2.waitKey(1)



    def perspective_image(self, img):

        # Color filter for the black platform
        # Good lighting
        lower = np.array([100,int(5*255/100.0),int(5*255/100.0)])
        upper = np.array([255,int(35*255/100.0),int(60*255/100.0)])
        
        # Darker lighting
        #lower = np.array([50,int(0*255/100.0),int(0*255/100.0)])
        #upper = np.array([255,int(30*255/100.0),int(40*255/100.0)])
        
        # Color filter for the green edge platform
        lower = np.array([50,int(25*255/100.0),int(45*255/100.0)])
        upper = np.array([100,int(50*255/100.0),int(65*255/100.0)])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lower,upper)
        #cv2.imshow("mask",mask)
        # cv2.waitKey(1)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:

            # check for the right contour
            r_max = 0
            for c in contours:
                _, radius = cv2.minEnclosingCircle(c)
                if radius > r_max:
                    r_max = radius
                    cnt = c
            epsilon = 0.04*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            approx = approx.tolist()
            #rospy.loginfo(len(approx)) 
            if len(approx) == 4 or len(approx) == 8:
                self.corner_filtering(approx)

            # This takes the corner_filter and reprojects the image
            pts1 = np.float32([self.corners[0],self.corners[1],self.corners[2],self.corners[3]])
            pts2 = np.float32([[0,0],[0,self.img_size],[self.img_size,self.img_size],[self.img_size,0]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(img,M,(self.img_size,self.img_size))
            #cv2.imshow("Image source",img)

            cv2.circle(img,(self.corners[0][0],self.corners[0][1]),5,[255,255,255],-1)
            cv2.circle(img,(self.corners[1][0],self.corners[1][1]),5,[0,255,255],-1)
            cv2.circle(img,(self.corners[2][0],self.corners[2][1]),5,[255,0,255],-1)
            cv2.circle(img,(self.corners[3][0],self.corners[3][1]),5,[255,255,0],-1)
            cv2.imshow("Source", img)
            #cv2.imshow("Projection",dst)
            cv2.waitKey(1)

            return dst


    def corner_filtering(self, pts):
        for i in range(len(self.corners)):
            self.corner_filter[i][1:10] = self.corner_filter[i][0:9]
            self.corner_filter[i][0] = pts[i][0]
            if np.abs(pts[i][0][0] - self.corners[i][0]) > 20:
                self.corners[i][0] = int(np.mean([item[0] for item in self.corner_filter[i]][0:10]))
                self.corners[i][1] = int(np.mean([item[1] for item in self.corner_filter[i]][0:10]))
                self.corner_filter[i][0] = self.corners[i]
            else:
                self.corners[i][0] = int(np.mean([item[0] for item in self.corner_filter[i]][0:5]))
                self.corners[i][1] = int(np.mean([item[1] for item in self.corner_filter[i]][0:5]))





if __name__ == '__main__':

    rospy.init_node('my_img_processor', anonymous=True)

    bt = ball_tracking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting Down")
