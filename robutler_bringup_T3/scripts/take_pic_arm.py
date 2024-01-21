#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw_arm', Image, self.image_callback)    # mudar camara
        self.save_folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'captured_images')
        os.makedirs(self.save_folder, exist_ok=True)
        rospy.loginfo("Image Subscriber Initialized")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("Image received successfully")
        except Exception as e:
            rospy.logerr(f"Error converting image: {str(e)}")
            return

        # Process the image as needed (e.g., display or save)
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"captured_image_cam2_{timestamp}.jpg"
        self.save_image(cv_image, filename)
        rospy.signal_shutdown("Image captured. Exiting...")

    def save_image(self, image, filename):
        filepath = os.path.join(self.save_folder, filename)
        cv2.imwrite(filepath, image)
        rospy.loginfo(f"Image saved as {filepath}")

def main():
    rospy.init_node('image_capture_node', anonymous=True)
    image_subscriber = ImageSubscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
