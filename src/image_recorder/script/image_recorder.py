#!/usr/bin/env python2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import cv2
# create the nodes


class ImageRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.img_callback)

    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite("/home/zhentian/TurtleBot/Images/"+ str(msg.header.seq)+".png", cv_image)

        return


if __name__ == '__main__':
    rospy.init_node("image_recorder")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = ImageRecorder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
