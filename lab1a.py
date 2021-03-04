import picar_4wd as fc
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.filedb import FileDB  
import time

speed = 30
ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []
us = Ultrasonic(Pin('D8'), Pin('D9'))
config = FileDB()  

ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)




def scan_step1(ref):
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = get_status_at(current_angle, ref1=ref)#ref1

    scan_list.append(status)
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False

def get_status_at(angle, ref1, ref2=10):
    dist = get_distance_at(angle)
    if dist > ref1 or dist == -2:
        return 0
    else:
        return 1

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    # print(distance)
    angle_distance = [angle, distance]
    return distance

def main():
    while True:
        scan_list = scan_step1(40)
        if not scan_list:
            continue
        tmp = scan_list[3:7]
        # if tmp == [0,0,0,0]:
        #     fc.backward(speed)

        if 1 not in tmp:
            print(tmp)
            fc.forward(speed)
        else:
            print(tmp)
            fc.stop()
            time.sleep(0.3)
            fc.backward(speed)
            time.sleep(0.3)
            fc.turn_right(speed)
            

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()