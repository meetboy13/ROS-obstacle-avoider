#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from assignment1.msg import name

class TakePhoto():

    # init name
    img_title = "obstacle.jpg"
    old_title = "obstacle.jpg"

    def __init__(self):
        rospy.init_node('take_photo', anonymous=False)
        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        rospy.Subscriber("/photo_name", name, self.updateName)

        # Allow up to one second to connection
        r = rospy.Rate(10);

        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # Take a photo

            if TakePhoto.old_title != TakePhoto.img_title:
                if TakePhoto.take_picture(self, TakePhoto.img_title):
                    rospy.loginfo("Saved image " + TakePhoto.img_title)
                else:
                    rospy.loginfo("No images received")
                TakePhoto.old_title = TakePhoto.img_title

            r.sleep()

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

    def updateName(self, data):
        # update name
        TakePhoto.img_title = data.name

if __name__ == '__main__':
    TakePhoto()
