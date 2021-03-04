import picar_4wd as fc
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.filedb import FileDB
from detect_picamera import main as detect
from path import search as search
import numpy as np
import time

speed = 10
turn_speed =5
turn_time_left =1.05
turn_time_right =1.05
turn_time =1
go_time =0.8
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
    start = [0, 0] # starting position
    end = [3,6] # ending position
    cost = 1 # cost per movement
    ind =0
    while True:
        if ind == 0:
            maze = [[0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0,0]]
            path = search(maze,cost, start, end)
        else:
            maze = new_maze
            path = search(maze,cost, [new_a,new_b], end)
        print("path now:",path)
        curn_node = cal_path(path)
        print("currnet noed",curn_node)

        # cal_path(path)
        if np.array_equal(curn_node[0], np.matrix(path[-2])) ==False:
            new_start = curn_node[0].reshape(-1,).tolist()
            print("############")
            print("new_start:",new_start)
            new_a =new_start[0][0]
            new_b=new_start[0][1]
            new_maze = maze
            print("rotation:",curn_node[1])
            if curn_node[1]==0:
                new_maze[new_a+1][new_b] =1
            elif curn_node[1]==1:
                fc.turn_right(turn_speed)
                time.sleep(turn_time_right)
                fc.stop()
                new_maze[new_a][new_b+1] =1
                print("adjust_L")
            elif curn_node[1]==-1:
                fc.turn_left(turn_speed)
                time.sleep(turn_time_left)
                fc.stop()
                new_maze[new_a][new_b-1] =1
                print("adjust_R")
            print(new_maze[0])
            print(new_maze[1])
            print(new_maze[2])
            print(new_maze[3])
            ind = ind+1
            continue
        else:
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
            print("done")
            break
    

    # path_return = path(maze,cost, start, end)
    # fc.forward(speed)
    # time.sleep(go_time)
    # cal_path(path_return)

def cal_path(path_return):
    zero_one =np.matrix([0,1])
    one_zero =np.matrix([1,0])
    mone_zero =np.matrix([-1,0])
    zero_mone =np.matrix([0,-1])
    rotation =[]
    for i in range(len(path_return)-1):
        cure = np.matrix(path_return[i])
        nextn =np.matrix(path_return[i+1])
        b = (nextn-cure)
        if sum(rotation) ==0 and (b==one_zero).all():
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break
            print("for")
            rotation.append(0)
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()    
        elif sum(rotation) ==0 and (b==zero_one).all():
            print("left")
            rotation.append(1)
            fc.turn_left(turn_speed)
            time.sleep(turn_time_left)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break    
                
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==0 and (b==zero_mone).all():
            print("right")
            rotation.append(-1)
            fc.turn_right(turn_speed)
            time.sleep(turn_time_right)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break  
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()    
        elif sum(rotation) ==1 and (b==one_zero).all():
            print("right")
            rotation.append(-1)
            fc.turn_right(turn_speed)
            time.sleep(turn_time_right)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break  
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==1 and (b==zero_one).all() :
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break
            print("for")
            rotation.append(0)
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==1 and (b==mone_zero).all():
            print("left")
            rotation.append(1)
            fc.turn_left(turn_speed)
            time.sleep(turn_time_left)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break    
                
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==-1 and (b==mone_zero).all():
            print("right")
            rotation.append(-1)
            fc.turn_right(turn_speed)
            time.sleep(turn_time_right)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break  
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==-1 and (b==zero_mone).all():
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break
            print("for")
            rotation.append(0)
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()
        elif sum(rotation) ==-1 and (b==one_zero).all():
            print("left")
            rotation.append(1)
            fc.turn_left(turn_speed)
            time.sleep(turn_time_left)
            fc.stop()
            for i in range(10):
                scan_list = scan_step1(25)
                if not scan_list:
                    continue
                tmp = scan_list[3:7]
                print(tmp)      
                if 1 in tmp:
                    print("cure",cure)
                    return cure,sum(rotation)
                    break    
                
            fc.forward(speed)
            time.sleep(go_time)
            fc.stop()                                              
    return cure,rotation



if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()