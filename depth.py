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

def get_mean_of_top_20(q):
    #get the distance
    inDisparity = q.get()  # blocking call, will wait until a new data has arrived
    frame = inDisparity.getFrame()
    # region_size = (200,320)
    region_size = (50,120)
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
        Last_mode = 'straight'
        mode = "straight"
        contact_class = 0
        while True:

            resized = jajucha.image_get(qRgb)


            mean_of_top_20 = get_mean_of_top_20(q)

            #print the distance
            print(mean_of_top_20)

            (V,L,R),resized = process.gridFront(resized)

            # print(V)
            # print(L)
            # print(R)



            L_V_set = [V[0],V[1],V[2]]
            R_V_set = [V[4],V[5],V[6]]

            T_center = 320 - (L[2] - R[2])
            steering_const = 0.06
            steering = (320 - T_center) * steering_const
            steering = int(round(steering))

            # print(abs(((L[1] + R[1])/2 + (L[0] + R[0])/2) - 440))

            if V[3] == 171 and abs(((L[1] + R[1])/2 + (L[0] + R[0])/2) - 440) < 100:# and (V[0] < V[1] < V[2] and V[4] > V[5] > V[6]):# and abs(V[0] - V[6]) < 86:
                mode = "straight"
                Last_mode = "straight"
            elif 171 not in V  and  V[6] < 156 and (sum(L_V_set) < sum(R_V_set)) and (171 not in [V[0],V[1],V[2]]): # and (V[6] < 156) and (V[3] <= 165) :
                mode = "R_curve"
                Last_mode = "R_curve"
            elif  171 not in V and V[3] < 160 and  V[1] < 156 and (sum(L_V_set) > sum(R_V_set)): #(171 not in [V[4],V[5],V[6]]) and (V[1] < 156) and (V[3] <= 158) 
                mode = "L_curve"
                Last_mode = "L_curve"

            if mean_of_top_20 >= 90 and mode != "L_curve":
                mode = "contact"
                contact_class += 1

            # ml.control(0, 0, 0, 9)
            # jajucha.image_send(resized,client_address)
            # continue

            # print(mode)


            if mode == "contact":
                while get_mean_of_top_20(q) > 20:
                    # print("contact")
                    ml.control(0, 0, -5, 9)
                else:
                    # time.sleep(0.5)
                    ml.control(0, 0, 7, 9)
                    # time.sleep(0.5)
                    mode = "default"
                    Last_mode = "straight"

                jajucha.image_send(resized,client_address)
                continue
            else:
                ml.control(0, 0, 12, 9)



            #print(f"mode:{mode}, left_arm: {left_arm}, right_arm: {right_arm}, velocity: {velocity}, contact_class: {contact_class}")


            jajucha.image_send(resized,client_address)

            mode = "default"



if __name__ == "__main__":
    unique_name = "user_program"
    set_unique_process_name(unique_name)

    jajucha = library.control.control()
    jajucha.control(45,45,50)

    main_loop()
    