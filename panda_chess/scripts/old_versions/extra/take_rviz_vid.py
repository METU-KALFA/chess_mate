#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2




def image_callback(image_msg):
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    video_writer.write(cv_image)


if __name__ == '__main__':
    rospy.init_node('image_recorder')
    bridge = CvBridge()
    video_writer = cv2.VideoWriter('/home/kovan-robot/recorded_video2.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20, (640, 480))  # Adjust parameters
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image,image_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        