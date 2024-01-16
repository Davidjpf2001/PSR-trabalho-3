#!/usr/bin/env python3 
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

def imgRead(img):

   # download required weight and config file of the yolo model here https://pjreddie.com/darknet/yolo/
    weight = "/home/david/catkin_ws/src/PSR-trabalho-3/robutler_bringup_T3/scripts/YOLO/yolov3.weights"
    cfg = "/home/david/catkin_ws/src/PSR-trabalho-3/robutler_bringup_T3/scripts/YOLO/darknet/cfg/yolov3.cfg"

    # give the configuration and weight files for the model and load the network
    yolo = cv2.dnn.readNet(weight, cfg)

    #coco name is the file which cointains all the 80 different object classes on which the yolo model is trained.
    with open("/home/david/catkin_ws/src/PSR-trabalho-3/robutler_bringup_T3/scripts/YOLO/darknet/data/coco.names", 'r') as f:
        classes = f.read().splitlines()
        
    # object to be detected
    obj = 'person'
    # find the id of the object to be detected    
    id1 = classes.index(obj)


    #img = cv2.imread(img)
    height, width, _ = img.shape
    # construct a blob from the image
    blob = cv2.dnn.blobFromImage(image = img, scalefactor = 1 / 255, size = (416, 416), mean = (0, 0, 0), swapRB=True, crop=False)
    # blob oject is given as input to the network
    yolo.setInput(blob)
    # get the index of the output layers
    output_layer_name = yolo.getUnconnectedOutLayersNames()
    # forward pass
    layeroutput = yolo.forward(output_layer_name)

    # post processing
    boxes = []
    confidences = []
    class_ids = []
    
    for output in layeroutput:
        for detection in output:
            score = detection[5:]
            class_id = np.argmax(score)
            confidence = score[class_id]
            if id1 == class_id:
            # filter out weak detections if probability is greater than the threshold
                if confidence > 0.05:
                    
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # find top left corner coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    
                    # get indexes of the object(s) detcted after supressing redundant bounding boxes
                    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.3)
                    # assign font
                    font = cv2.FONT_HERSHEY_COMPLEX

                    # assign colors to the bounding boxes
                    colors = np.random.uniform(0, 255, size=(len(boxes), 3))
                    
                    # add bounding boxes to each object in the image frame
                    print("Número de detecções:", len(indexes.flatten()))
                    try:
                        for i in indexes.flatten():
                            x, y, w, h = boxes[i]
                            label = str(classes[class_ids[i]])
                            color = colors[i]
                            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                            cv2.putText(img, f'Obj: {obj}', (x, y-5), font, 0.5, color, 2)
                            cv2.imshow('img', img)
                            cv2.waitKey(1)

                    except:
                        pass

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.save_folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'captured_images')
        os.makedirs(self.save_folder, exist_ok=True)
        rospy.loginfo("Imagem Inicializada")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("Imagem recebida com sucesso")
            #cv2.imshow('test', cv_image)
            imgRead(cv_image)
            

        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem: {str(e)}")

def main():
    rospy.init_node('node_de_captura_de_imagem', anonymous=True)
    image_subscriber = ImageSubscriber()

    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.loginfo("Node de Captura de Imagem Encerrado")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()