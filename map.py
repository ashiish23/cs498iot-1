import picar_4wd as fc
import numpy as np
import time
import functools
import cv2

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


# If you are centered in a box of dimensions 2M x 2M
#  and cast a ray out at angle_rad
# This function will return the coordinate pair (y,x)
#  that will intersect the box first
def boundaryDetector(angle_rad, M):
	cos = np.cos(angle_rad)
	sin = np.sin(angle_rad)

	if np.abs(sin) < 1e-6: d = np.abs(1./cos)
	elif np.abs(cos) < 1e-6: d = np.abs(1./sin)
	else: d = min(np.abs(1./cos), np.abs(1./sin))
	return (M - M*d*sin, M + M*d*cos)

# Creates a handy backdrop 
#  that is useful for visualizing ultrasonic points
def setBackground(N = 139):
	m = np.zeros((N, N, 3), 'int32')
	M = N // 2
	m[M, M] = (255, 255, 255)
	#m = cv2.line(m, (0, M), (N, M), (255, 255, 255), 1)
	#m = cv2.line(m, (M, 0), (M, N), (255, 255, 255), 1)

	# Draw sensor range
	#angleStart = np.round(boundaryDetector(np.radians(SERVO_OFFSET), M)).astype('int32')
	#m = cv2.line(m, (M, M), (angleStart[0], angleStart[1]), (255, 255, 255), 1)
	#angleStop = np.round(boundaryDetector(np.radians(180 + SERVO_OFFSET), M)).astype('int32')
	#m = cv2.line(m, (M, M), (angleStop[0], angleStop[1]), (255, 255, 255), 1)

	for i in range(0, 17):
		angle = SERVO_OFFSET + i*180./16
		power = 1
		if (i % 2) == 0: power += 2
		if (i % 4) == 0: power += 4
		if (i % 8) == 0: power += 8
		level = (1 << power) 
		angleStop = np.round(boundaryDetector(np.radians(angle), M)).astype('int32')
		m = cv2.line(m, (M, M), (angleStop[1], angleStop[0]), (level, level, level), 1)

	return m


def sweep(points  = 12, precise = False, gridsize = 139):
	#m = setBackground(gridsize)
	m = np.zeros((gridsize, gridsize, 3), 'int32')
	distances = {}
	maxD = -2	# The biggest ditance we've seen so far
	maxA = 0	# The angle that produced the biggest distance
	lastReadingGood = False
	state = 0
	arcPoints = []
	arcs = []
	delta = np.radians(180./(2*points))

	def goodReading(d, angle_rad):
		dx = (gridsize//2) + int(d * np.cos(angle_rad))
		dy = (gridsize//2) - int(d * np.sin(angle_rad))
		return d < 80 and (dx >= 0 and dx < gridsize) and (dy >= 0 and dy < gridsize)

	def state0(d, angle_rad, arcPoints):
		if goodReading(d, angle_rad):
			arcPoints = [ (angle_rad - delta / 2, d), (angle_rad, d), (angle_rad + delta / 2, d) ]
			return state1, arcPoints, None
		else:
			return state0, arcPoints, None

	def state1(d, angle_rad, arcPoints):
		if goodReading(d, angle_rad):
			# The last entry in arc points is always an interpolation
			#  and this new reading will give us more information to make a better interpolation
			lastReading = arcPoints[-1]
			arcPoints.pop()
			arcPoints = arcPoints + [ (angle_rad - delta / 2,
				(d + lastReading[1])/2),
				(angle_rad, d),
				(angle_rad + delta / 2, d) ]
			return state1, arcPoints, None
		else:
			#print("Found closed arc: {0}".format(arcPoints))
			return state0, [], arcPoints

	state = state0
	RED = (0, 0, 255)
	BLUE = (255, 0, 0)
	GREEN = (0, 255, 0)
	YELLOW = (2550, 255, 0)
	GRAY = (128,128,128)

	servo_angles = np.linspace(-90 + SERVO_OFFSET, 90 + SERVO_OFFSET, points, True)
	for servo_angle in servo_angles:
		# The servo goes from looking to the right @ -90 to left @ 90
		# remap this to right @ 0 to left @ 180
		angle_rad = np.radians(servo_angle) + np.pi / 2
		setAngle(servo_angle)
		d = getDistance(precise)

		print("servo angle {0:5.1f} distance {1:5.3f} state {2}".format(servo_angle, d, state))
		dx = (gridsize//2) + int(d * np.cos(angle_rad))
		dy = (gridsize//2) - int(d * np.sin(angle_rad))
		if (dx >= 0 and dx < gridsize) and (dy >= 0 and dy < gridsize):
			m[dy,dx] = GREEN

		nextState, arcPoints, arcFinal = state(d, angle_rad, arcPoints)
		if arcFinal != None: arcs = arcs + [arcFinal]
		state = nextState

	# Force DFA to close out arc
	nextState, arcPoints, arcFinal = state(9999, angle_rad, arcPoints)	
	print("last")
	print(arcFinal)
	if arcFinal != None: arcs = arcs + [arcFinal]
	print(arcs)

	print(len(arcs))
	for arc in arcs:
		print("boom")

	# For arc bein and end, we want to draw a ray to infinity (on our grid).  To do this
	# first need to figure out what rasterize points
	for arc in arcs:
		N = len(arc)
		pts = np.zeros((N*2, 2), dtype='int32')
		idx = 0
		points = []
		endPoints = []
		for angle_rad, d in arc:
			dx = (gridsize//2) + int(d * np.cos(angle_rad))
			dy = (gridsize//2) - int(d * np.sin(angle_rad))
			pts[idx] = (dx, dy)
			e = boundaryDetector(angle_rad, gridsize//2)
			e = np.round(e).astype('int32')
			pts[2*N-idx-1] = (e[1], e[0])
			idx = idx + 1
		print("new arc\n==========")
		pts = pts.reshape(-1, 1, 2)
		print(pts)
		print("end arc\n==========\n")
		m = cv2.fillConvexPoly(m, pts, GRAY, 1)

	setAngle(0)
	m[gridsize//2, gridsize//2] = (255, 255, 255)
	return m

def test():
	N = 139
	m = sweep(50, gridsize = N)
	cv2.imwrite("test.png", m)
	fc.forward(10)
	time.sleep(0.5)
	fc.forward(0)

	# our predicted n is m shifted vertically + 10
	mprime = np.zeros(m.shape, m.dtype)
	mprime[10:, :] = m[0:(N-10), :]
	cv2.imwrite("test2-guess.png", mprime)

	n = sweep(50)
	cv2.imwrite("test2.png", n)

	diff = np.abs(n - mprime)
	cv2.imwrite("diff.png", diff)

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

if __name__ == '__main__':
	speed = 10
	clear = 20
	tick = 0
	state = state1

	while tick < 1000:
		sweep()

