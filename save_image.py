import sys
import os.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
import library.control
import library.img_process as process
import numpy as np
import depthai as dai
import cv2
import setproctitle
import motorlib as ml
import time
import math
from datetime import datetime
def get_mean_of_top_20(q):
    #get the distance
    inDisparity = q.get()  # blocking call, will wait until a new data has arrived
    frame = inDisparity.getFrame()
    # region_size = (200,320)
    region_size = (50,80)
    center = (200,320)
    start = (center[0]-region_size[0] // 2, center[1] - region_size[1]//2)
    end = (start[0]+region_size[0],start[1]+region_size[1])
    region = frame[start[0]:end[0],start[1]:end[1]]
    top_20_values = np.partition(region.flatten(),-20)[-20:]
    return np.mean(top_20_values)
    

def set_unique_process_name(unique_name):
    setproctitle.setproctitle(unique_name)

def main_loop():
    with dai.Device(jajucha.pipeline) as device:
        qRgb = device.getOutputQueue(name = 'rgb', maxSize = 4,blocking = False)
        q = device.getOutputQueue(name="disparity", maxSize=1, blocking = False)
        print("jajuchaready") #do not delete this line
        data, client_address = jajucha.udp_sock.recvfrom(1024)
        print("jajucharunning")

        now_class = 'normal'  # 일반 도로
        # now_class = 'obstacle'  # 장애물
        # now_class = 'stopper'  # 차단기

        img_path = '~/saved_image/' + now_class

        contact_class = 0
        os.makedirs(img_path, exist_ok=True)
        while True:

            resized = jajucha.image_get(qRgb)
            
            cv2.imwrite(img_path+'/'+datetime.now().strftime("%Y-%m-%d %H:%M:%S-%f")+'.jpg', resized)

            (V,L,R),resized = process.gridFront(resized)
            
            T_center = 320 - (L[2] - R[2])
            steering_const = 0.05
            steering = (320 - T_center) * steering_const
            steering = int(round(steering))

            if now_class == 'normal':  # 일반 공도 주행
                
                L_V_set = [V[0],V[1],V[2]]
                R_V_set = [V[4],V[5],V[6]]
            
                if V[3] == 171 and abs(((L[1] + R[1])/2 + (L[0] + R[0])/2) - 440) < 100 and (V[0] < V[1] < V[2] and V[4] > V[5] > V[6]):# and abs(V[0] - V[6]) < 86:
                    mode = "straight"
                    Last_mode = "straight"
                elif (171 not in [V[0],V[1],V[2]]) and (V[6] < 156) and (V[3] <= 165) and (sum(L_V_set) < sum(R_V_set)):
                    mode = "R_curve"
                    Last_mode = "R_curve"
                elif (171 not in [V[4],V[5],V[6]]) and (V[1] < 156) and (V[3] <= 158) and (sum(L_V_set) > sum(R_V_set)):
                    mode = "L_curve"
                    Last_mode = "L_curve"
    
                if mode == 'default': 
                    mode = Last_mode
                    print("default", end=' ')
                
                if mode == 'straight':
                    if V[3] <= 150 and False:
                        left_arm = -steering
                        right_arm = -steering
                        velocity = 7
                    else:
                        left_arm = -steering
                        right_arm = -steering
                        velocity = 12
                elif mode == 'R_curve':
                    if V[3] <= 95:
                        left_arm = 25
                        right_arm = 25
                        velocity = 6
                    else:
                        left_arm = 30
                        right_arm = 30
                        velocity = 6
                elif mode == 'L_curve':
                    if V[3] <= 95:
                        left_arm = -25
                        right_arm = -25
                        velocity = 6
                    else:
                        left_arm = -40
                        right_arm = -40
                        velocity = 6
            else:  # 기타(차, 차단기)
                left_arm = -steering
                right_arm = -steering
                velocity = 5
                
            print(f"left_arm: {left_arm}, right_arm: {right_arm}, velocity: {velocity}, contact_class: {contact_class}")

            ml.control(left_arm, right_arm, velocity, 9)

            jajucha.image_send(resized,client_address)



if __name__ == "__main__":
    unique_name = "user_program"
    set_unique_process_name(unique_name)

    jajucha = library.control.control()
    jajucha.control(45,45,50)

    main_loop()
