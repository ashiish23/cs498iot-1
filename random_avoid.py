## CS498 IOT Lab 1 Part 1
## rdb4/rdbisch@gmail.com
import picar_4wd as fc
import numpy as np
import time
import functools
import random

SERVO_OFFSET = 8

def setAngle(angle):
	fc.servo.set_angle(angle - SERVO_OFFSET)
	time.sleep(0.04) # Constant from picar_4wd/__init__.py 

def clean(ds):
	def C(x):
		return x
		if x < 0 or x > 100: return 100
		else: return x
	return [ C(x) for x in ds ]

def getDistance(precise = False):
	if not precise:
		return my_getDistance()
	else:
		trials = 5
		while True:
			readings = [my_getDistance() for i in range(trials)]
			readings = clean(readings)
			mean = np.mean(readings)
			sd = np.std(readings)
			if sd > 1 and trials < 17: trials += 2
			else: return mean 

def my_getDistance():
	fc.us.trig.low()
	time.sleep(0.01)
	fc.us.trig.high()
	time.sleep(0.000015)
	fc.us.trig.low()
	pulse_end = 0
	pulse_start = 0
	while fc.us.echo.value() == 0: continue
	pulse_start = time.time()
	while fc.us.echo.value() == 1: continue
	pulse_end = time.time()
	delta = pulse_end - pulse_start
	cm = (343.0/2.0) * (100.0) * delta  # m/s * (cm/m) * time = cm
	#if cm > 150: cm = 0   
	return cm

def turnAngle(angle):
	if (angle > 0): turnLeftAngle(angle)
	else: turnRightAngle(-angle)

def turnLeftAngle(angle):
	#2.78 by experimentation.
	duration = (angle /180.) * (1.83333)
	fc.turn_left(10)
	time.sleep(duration)
	fc.turn_left(0)

def turnRightAngle(angle):
	#2.78 by experimentation.
	duration = (angle / 180.) * (1.83333)
	fc.turn_right(10)
	time.sleep(duration)
	fc.turn_right(0)


def state1(tick, clear = 20, speed = 10):
	setAngle(-45)
	dR = getDistance(False)
	setAngle(0)
	d = getDistance(False)
	setAngle(45)
	dL = getDistance(False)
	print("tick {0} state 1 fwd distance [{2:5.3f} {1:5.3f} {3:5.3f}]".format(tick, d, dL, dR))

	if d < clear or dL < clear or dR < clear: 
		fc.forward(0)
		return state2
	else:
		fc.forward(speed)
		return state1

def state2(tick, clear = 20, speed = 10):
	fc.backward(speed)
	time.sleep(0.50)
	fc.backward(0)

	# pick random angle 
	angles = [22.5*i for i in range(1, 16)]
	angle = random.choice(angles)	
	if angle > 180: angle = angle - 360 
	
	print("tick {0} state 2 rotating angle {1:5.2f}".format(tick, angle))
	turnAngle(angle)
	return state1

def stateStop(tick):
	fc.stop()

if __name__ == '__main__':
	speed = 10
	clear = 20
	tick = 0
	state = state1

	while tick < 150:
		nextState = state(tick, clear, speed)
		tick = tick + 1
		state = nextState
	stateStop(tick)

