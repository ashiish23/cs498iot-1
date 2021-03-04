import picar_4wd as fc
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.filedb import FileDB  
import time
import cv2 as cv
import numpy as np
import math 

speed = 30
ANGLE_RANGE = 180
STEP = 10
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []
angle_distance_list =[]
us = Ultrasonic(Pin('D8'), Pin('D9'))
config = FileDB()  

ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)




point_size = 1
point_color = (0, 0, 255) # BGR
yellow = (51, 255, 255) # BGR
thickness = 4 



def scan_step1(ref):
    global angle_distance_list, scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = get_status_at(current_angle, ref1=ref)#ref1
    scan_list.append(status)
    angle_distance_list.append(angle_distance)
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
            angle_distance_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        tmp1 = angle_distance_list.copy()
        angle_distance_list = []
        return tmp,tmp1
    else:
        return False

def get_status_at(angle, ref1):
    dist = get_distance_at(angle)
    if dist > ref1 or dist == -2:
        print("0000000000")
        return 0
    else:
        print("1111111111")
        return 1

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    print(angle_distance)
    return distance


def draw_dot(scan_list,a_d_list,img):


    for i in range(len(scan_list)):
        if scan_list[i]==1:
            angle = 90- (a_d_list[i][0])
            length = a_d_list[i][1]+50
            P1 =[240,100]
            P2=[]

            P2.append(int(round(P1[0] + length * math.cos(angle * math.pi / 180.0))))
            P2.append(int(round(P1[1] + length * math.sin(angle * math.pi / 180.0))))

            cv.circle(img, (P2[0],P2[1]), point_size, point_color, thickness)

            # for i in a_d_list:

    cv.imshow('image', img)
    cv.waitKey(500)

def main():
    while True:
        scan_list = scan_step1(33)
        if not scan_list:
            continue
        tmp = scan_list[0][3:7]
        # print(angle_distance_list)
        # if tmp == [0,0,0,0]:
        #     fc.backward(speed)
        print(tmp)
        img = np.zeros((480, 480, 3), np.uint8) 
        cv.rectangle(img, (190,10 ), (290,100), yellow, 2)
        cv.namedWindow("image")
        cv.imshow('image', img)
        cv.waitKey(500)
        draw_dot(scan_list[0],scan_list[1],img)
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        if tmp != [2,2,2,2] :
            fc.stop()
            # time.sleep(0.3)
            # fc.backward(speed)
            # time.sleep(0.3)
            # fc.turn_right(speed)

        else:
            fc.stop()
            # fc.forward(speed)

if __name__ == "__main__":



    try: 
        main()

    finally: 
        fc.stop()
    