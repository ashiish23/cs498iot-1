import picar_4wd as fc
import numpy as np
import time
import functools

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

def sweep(delta = 15, precise = False):
	# My servo is mounted wrong by a tooth? So I adjust 
    # by 8 degrees

	distances = {}
	maxD = -2	# The biggest ditance we've seen so far
	maxA = 0	# The angle that produced the biggest distance
	angle = -90 + SERVO_OFFSET
	while (angle <= 90 + SERVO_OFFSET):
		setAngle(angle)
		d = getDistance(precise)
		if d > maxD:
			maxD = d
			maxA = angle
		distances[angle] = d
		angle = angle + delta
	setAngle(0)
	return { "max": maxA, "sweep": distances, "delta": delta, "precise": precise }

def turnAngle(angle):
	if (angle > 0): turnLeftAngle(angle)
	else: turnRightAngle(-angle)

def turnLeftAngle(angle):
	#2.78 by experimentation.
	duration = (angle /180.) * (2.2)
	fc.turn_left(10)
	time.sleep(duration)
	fc.turn_left(0)

def turnRightAngle(angle):
	#2.78 by experimentation.
	duration = (angle / 180.) * (2.2)
	fc.turn_right(10)
	time.sleep(duration)
	fc.turn_right(0)


def state1(tick, clear = 20, speed = 10, lastState1Distance = None):
	setAngle(0)
	d = getDistance(True)
	print("tick {0} state 1 fwd distance {1:5.3f}".format(tick, d))

	if lastState1Distance != None:
		# We should be getting smaller, so
		diff = lastState1Distance - d
		if diff > 20 or diff < 10:
			# We expect to move about 15-18 cm each call to state 1
			# Drastic changes in distance sensed on consecutive state 1 implies
			# an inaccurate reading or a new obstacle. 
			# In either case the right thing to do is to stop and rescan.
			print("!!! Distance out of pattern !!!")
			return state2
#		else: print("diff {0} within tol".format(diff))

	# Experimental.  We've cleared straight ahead, but our car is a hog.
	# We need to make sure the space in front of us is wide enough.  The car is about 16cm
	# and we know that we travel around that far a the current speed.  So we will also scan 
	# in a short arc in front of us too.  It turns out that the angle we need is -45 to 45.  We need
	# these to all be >= d at angle 0.  To save time we'll only check end points
	#setAngle(45)
	#dL = getDistance()
	#setAngle(-45)
	#dR = getDistance()
	#print("arc check {0:5.3f}, {1:5.3f}, {2:5.3f}".format(dL, d, dR))
	if (d < clear): #or (dL < clear) or (dR < clear):
		return state3
	else:
		fc.forward(speed)
		time.sleep(0.5)
		fc.forward(0)
		# construct a new 'state1' function with the
		#  last distance passed in.
		return functools.partial(state1, lastState1Distance = d)

def state3(tick, clear = 20, speed = 10):
	print("tick {0} state 3".format(tick))
	fc.backward(speed)
	time.sleep(0.25)
	fc.backward(0)
	return functools.partial(state2, escape = state4)

def state4(tick, clear = 20, speed = 10):
	print("tick {0} state 4".format(tick))
	turnAngle(90)
	return state2

def state2(tick, clear = 20, speed = 10, escape = state3):
	angle_sweep = 15
	# Distance threshold based off car width=16cm
	dt = 8 / np.sin(angle_sweep * np.pi / 180.)
	print("tick {0} state 2".format(tick))
	swept = sweep(angle_sweep, False)["sweep"]
	angles = list(swept.keys())
	angles.sort(key = lambda x: abs(x))
	choosen_angle = None

	for angle in angles[7:]:
		#print("Swept angle is {0} with distance {1}.. This gives theta >= {2}.".format(angle,
		#	swept[angle], (180./np.pi) * np.arctan2(8, swept[angle])))

		#theta = np.arctan2(8, swept[angle])
		#theta = theta * 180. / np.pi
		#if theta < 2*angle_sweep:
		if swept[angle] > dt:
			choosen_angle = angle
			break
	
	if choosen_angle == None:
		return escape
	
	else:
		print("Turning {0} degrees".format(choosen_angle))
		turnAngle(choosen_angle)
		return state1


if __name__ == '__main__':
	speed = 10
	clear = 20
	tick = 0
	state = state1

	while tick < 1000:
		nextState = state(tick, clear, speed)
		tick = tick + 1
		state = nextState

