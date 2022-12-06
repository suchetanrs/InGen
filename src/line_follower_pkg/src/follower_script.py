#!/usr/bin/env python

# This program is to be used with line_follower_pkg and yellow_line.world. Once yellow_line.world
# #is up and runnin and this program is launched, the turtlebot will begin to follow the yellow line
# in the gazebo world.
# Please note; not happting with the whole shutdown process, in particular the use of 'gloal ctrl_c'

import roslib
import sys
import rospy
import cv2
import numpy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

ctrl_c = False


class LineFollower:

    def __init__(self):

        self.bridge_object = CvBridge()
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)
        # publish to /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist_object = Twist()

    def clean_up(self):
        # this function runs upon shutdown.
        cv2.destroyAllWindows()
        rospy.loginfo("Unregistering Camera Subscriber")
        self.image_sub.unregister()
        rospy.loginfo("Wait 1 second")
        rospy.sleep(1)
        rospy.loginfo("Stopping Motor")
        self.twist_object.linear.x = 0.0
        self.twist_object.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_object)
        rospy.loginfo("Angular turning Value Sent = " +
                      str(self.twist_object.angular.z))
        rospy.loginfo("Unregistering cmd_vel Publisher")
        self.cmd_vel_pub.unregister()
        rospy.loginfo("Wait 1 second")
        rospy.sleep(1)

    def camera_callback(self, data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        #### 1.GET IMAGE INFO AND CROP ####
        # get info about shape of captured image
        height, width, channels = cv_image.shape
        #print (cv_image.shape)
        # Crop image to only see 100 rows
        descentre = 160
        rows_to_watch = 100
        crop_img = cv_image[(height)/2+descentre: (height) /
                            2+(descentre+rows_to_watch)][1:width]
        #print (crop_img.shape)

        #### 2.GET IMAGE INFO AND CROP ####
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)


        #CUSTOM CODE BEGINS
        print(hsv[-1][0]) #first index is for column element, 2nd index is for row element
        print("******************")
        flag_start=False
        flag_end=False
        start_point=0
        end_point=0
        middle_point=0

        color_val=hsv[-1][0][2]
        for i in range(0,1920):
            if(flag_start==False and hsv[-1][i][2]>100):
                print(hsv[-1][i][2])
                print("index start {}").format(i)
                start_point=i
                flag_start=True
            if(flag_start==True and flag_end==False and hsv[-1][i][2]<100):
                print(hsv[-1][i][2])
                print("index end {}").format(i)
                end_point=i
                flag_end=True
        
        middle_point=(start_point+end_point)/2
        print("Middle point {}").format(middle_point)

        if(middle_point<940):
            self.twist_object.linear.x =0.0
            self.twist_object.angular.z=0.1
        elif(middle_point>980):
            self.twist_object.linear.x =0.0
            self.twist_object.angular.z=-0.1
        else:
            self.twist_object.linear.x =0.1
            self.twist_object.angular.z=0.0

        #CUSTOM CODE ENDS
        cv2.imshow("HSV Image", hsv)
        cv2.waitKey(1)
        self.cmd_vel_pub.publish(self.twist_object)


def main():

    global ctrl_c
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        rospy.loginfo("Initiating ShutDown")
        line_follower_object.clean_up()
        rospy.loginfo("ShutdownTime!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep


if __name__ == '__main__':
    main()
