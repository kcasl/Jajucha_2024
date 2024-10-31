import numpy as np
import cv2

# x,y = center , width,height = box size

def traffic_light(img_color,x,y,width,height):

    #extract ROI from the image
    img_color = img_color[y-int(height/2):y+int(height/2), x-int(width/2):x+int(width/2)]
    
    #convert image channel to hsv
    img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV) 

    #hyperparameter
    lower_red_1 = (169, 40, 40) 
    upper_red_1 = (179, 255, 255)

    lower_red_2 = (0, 40, 40) 
    upper_red_2 = (10, 255, 255)

    lower_green = (50, 40, 40) 
    upper_green = (70, 255, 255)

    #mask image - red    
    img_mask_red_1 = cv2.inRange(img_hsv, lower_red_1, upper_red_1) 
    img_mask_red_2 = cv2.inRange(img_hsv, lower_red_2, upper_red_2)
    
    #mask image - green
    img_mask_green = cv2.inRange(img_hsv, lower_green, upper_green) 

    #calcualte whether green or red
    red_count = np.sum(img_mask_red_1 == 255)
    red_count += np.sum(img_mask_red_2 == 255)
    green_count = np.sum(img_mask_green == 255)
    
   # print("red",red_count,"green",green_count)
    if(red_count > 10000):
        return 1
    else:
        return 0
