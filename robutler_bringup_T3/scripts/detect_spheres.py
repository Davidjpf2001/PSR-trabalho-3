#!/usr/bin/env python3 
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from ultralytics import YOLO
import math 

def detect_spheres(img):

    max_B = 0
    min_B = 0

    max_G = 0
    min_G = 0

    max_R = 255
    min_R = 101

    upper = np.array([max_B, max_G, max_R])
    lower = np.array([min_B, min_G, min_R])
    
	# Mask Creation
    image_rgb = cv2.inRange(img, lower, upper)
    
    #Showing changed Image
    cv2.imshow('TrackBar', image_rgb)
    cv2.waitKey(1)

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw_arm', Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detect_spheres(cv_image)
            

        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem: {str(e)}")

def main():
    rospy.init_node('node_de_deteção', anonymous=True)
    image_subscriber = ImageSubscriber()

    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.loginfo("Node de Captura de Imagem Encerrado")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()