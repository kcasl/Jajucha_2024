import time

import serial


def control(left_arm, right_arm, velocity, mode):
    # velocity: 0 +- 50
    # left/right_arm: 0 +- 23
    
    velocity = 50 - velocity
    left_arm = 45 - left_arm
    right_arm = 45 - right_arm

    if velocity > 90:
        velocity = 90
    elif velocity < 10:
        velocity = 10

    if left_arm > 68:
        left_arm = 68
    elif left_arm < 22:
        left_arm = 22

    if right_arm > 68:
        right_arm = 68
    elif right_arm < 22:
        right_arm = 22

    # print(left_arm, right_arm)


    lat = (int)(left_arm / 10)
    lao = left_arm - lat * 10

    rat = (int)(right_arm / 10)
    rao = right_arm - rat * 10

    vt = (int)(velocity / 10)
    vo = velocity - vt * 10

    mt = (int)(mode / 10)
    mo = mode - mt * 10

    li = [str(lat), str(lao), str(rat), str(rao), str(vt), str(vo), str(mt), str(mo)]
    li = ''.join(li)

    uart.write(str.encode(li))
    time.sleep(0.05)


def init():
    control(0, 0, 0, 9)


def stop():
    control(0, 0, 0, 0)
    
def test():
    control(50,50,0,9)


port = '/dev/ttyTHS2'  # serial port
baud = 576000  # serial baudrate
uart = serial.Serial(port, baud, timeout=0)
init()
time.sleep(1)
test()
