#!/usr/bin/env python3 
import cv2 
import numpy as np
import json
import sys

#######################################
# Código desenvolvido na UC de PSR


# Definition of onTrackBar function responsible for segmenting pixels
# in a given for R, G and B
def onTrackbar(image_rgb, min_B, min_G, min_R, max_B, max_G, max_R):

    # Creation of array with max and min of each channel BGR
    upper = np.array([max_B, max_G, max_R])
    lower = np.array([min_B, min_G, min_R])
    
	# Mask Creation
    image_rgb = cv2.inRange(image_rgb, lower, upper)
    
    #Showing changed Image
    cv2.imshow('TrackBar', image_rgb)
    cv2.imwrite('Color_segmenter.png', image_rgb)   

# nothing function, responsible to letting createTrackBar continue  
def nothing(x):
    pass
    

def main():

    imagem = cv2.imread('/home/david/Imagens/Teste2.png')
	
	#window name
    cv2.namedWindow('TrackBar',cv2.WINDOW_NORMAL)

	# creating trackbars for each min max channel
    cv2.createTrackbar('min B', 'TrackBar', 0, 255, nothing) 
    cv2.createTrackbar('max B', 'TrackBar', 0, 255, nothing)
    cv2.createTrackbar('min G', 'TrackBar', 0, 255, nothing) 
    cv2.createTrackbar('max G', 'TrackBar', 0, 255, nothing)
    cv2.createTrackbar('min R', 'TrackBar', 0, 255, nothing) 
    cv2.createTrackbar('max R', 'TrackBar', 0, 255, nothing)

    while(True):

        #saving position of each trackbar
        max_BH = cv2.getTrackbarPos("max B", "TrackBar")
        min_BH = cv2.getTrackbarPos("min B", "TrackBar")
        max_GS = cv2.getTrackbarPos("max G", "TrackBar")
        min_GS = cv2.getTrackbarPos("min G", "TrackBar")
        max_RV = cv2.getTrackbarPos("max R", "TrackBar")
        min_RV = cv2.getTrackbarPos("min R", "TrackBar")
        
        # break if user press q (quit) or w to write values on file
        k = cv2.waitKey(1) & 0xFF
        if k == 113 or k == 119:
            if k == 113:
                print("q key pressed, exiting...")
            break
        
        # showing the video
        cv2.imshow('Original Image', imagem)

        # ontrackbar function call to update the frame
        onTrackbar(imagem, min_BH, min_GS, min_RV, max_BH, max_GS, max_RV)
    
    # close all windows
    cv2.destroyAllWindows()
    
    # np.savez('limits_RGB.npz', min_R = min_RV , max_R = max_RV, 
    #          min_G = min_GS , max_G = max_GS, 
    #          min_B = min_BH , max_B = max_BH)
    
if __name__ == '__main__':
    main()