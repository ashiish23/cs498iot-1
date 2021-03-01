import picar_4wd as fc
import numpy as np
import time
import functools
import cv2
import picamera

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
	time.sleep(0.1)
	fc.us.trig.high()
	time.sleep(0.00001)
	fc.us.trig.low()
	pulse_end = 0
	pulse_start = 0
	while fc.us.echo.value() == 0: pulse_start = time.time()
	while fc.us.echo.value() == 1: pulse_end = time.time()
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

def sweep(points  = 12, precise = False, gridsize = 201, debug = False):
	m = setBackground(gridsize)
	#m = np.zeros((gridsize, gridsize, 3), 'int32')
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
		return d < 200 and (dx >= 0 and dx < gridsize) and (dy >= 0 and dy < gridsize)

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
	CYAN = (255, 255, 0)
	GRAY = (128,128,128)
	YELLOW = (0, 255, 255)

	servo_angles = np.linspace(-90 + SERVO_OFFSET, 90 + SERVO_OFFSET, points, True)
	for servo_angle in servo_angles:
		# The servo goes from looking to the right @ -90 to left @ 90
		# remap this to right @ 0 to left @ 180
		angle_rad = np.radians(servo_angle) + np.pi / 2
		setAngle(servo_angle)
		d = getDistance(precise)

		if debug: print("servo angle {0:5.1f} distance {1:5.3f} state {2}".format(servo_angle, d, state))
		dx = (gridsize//2) + int(d * np.cos(angle_rad))
		dy = (gridsize//2) - int(d * np.sin(angle_rad))
		if debug: print("dx={0:5.3f} dy={1:5.3f}".format(dx,dy))
		if (dx >= 0 and dx < gridsize) and (dy >= 0 and dy < gridsize):
			m[dy,dx] = GREEN

		nextState, arcPoints, arcFinal = state(d, angle_rad, arcPoints)
		if arcFinal != None: arcs = arcs + [arcFinal]
		state = nextState

	# Force DFA to close out arc
	nextState, arcPoints, arcFinal = state(9999, angle_rad, arcPoints)	
	if debug:
		print("last")
		print(arcFinal)
	if arcFinal != None: arcs = arcs + [arcFinal]

	# For arc bein and end, we want to draw a ray to infinity (on our grid).  To do this
	# first need to figure out what rasterize points
	for arc in arcs:
		N = len(arc)
		pts = np.zeros((N, 2), dtype='int32')
		#pts = np.zeros((N*2, 2), dtype='int32')
		idx = 0
		points = []
		endPoints = []
		for angle_rad, d in arc:
			dx = (gridsize//2) + int(d * np.cos(angle_rad))
			dy = (gridsize//2) - int(d * np.sin(angle_rad))
			pts[idx] = (dx, dy)
			#e = boundaryDetector(angle_rad, gridsize//2)
			#e = np.round(e).astype('int32')
			#pts[2*N-idx-1] = (e[1], e[0])
			idx = idx + 1
		#m = cv2.fillConvexPoly(m, pts, GRAY, 1)
		#m = cv2.polylines(m, [pts], False, YELLOW, 2)
		if debug:
			print("new arc\n==========")
			pts = pts.reshape(-1, 1, 2)
			print(pts)
			print("end arc\n==========\n")

	setAngle(0)
	m[gridsize//2, gridsize//2] = (255, 255, 255)
	return m

def test():
	#camera = PiCamera()
	N = 139
	m = sweep(50, gridsize = N)
	cv2.imwrite("test.png", m)

	# forward movement ####
	if None:
		fc.forward(10)
		time.sleep(0.5)
		fc.forward(0)
		# our predicted n is m shifted vertically + 10
		mprime = np.zeros(m.shape, m.dtype)
		mprime[10:, :] = m[0:(N-10), :]
		cv2.imwrite("test2-guess.png", mprime)

	# Rotate left
	elif None:
		image_center = tuple(np.array(m.shape[1::-1]) / 2)
		rot_mat = cv2.getRotationMatrix2D(image_center, -15, 1.0)
		mm = m.astype('float')/256.0
		mprime = cv2.warpAffine(mm, rot_mat, m.shape[1::-1], flags=cv2.INTER_LINEAR)
		mprime = (256*mprime).astype('int32')
		cv2.imwrite("test2-guess.png", mprime)
		turnLeftAngle(15)
		n = sweep(50)
		cv2.imwrite("test2.png", n)

	# Rotate Right
	image_center = tuple(np.array(m.shape[1::-1]) / 2)
	rot_mat = cv2.getRotationMatrix2D(image_center, 15, 1.0)
	mm = m.astype('float')/256.0
	mprime = cv2.warpAffine(mm, rot_mat, m.shape[1::-1], flags=cv2.INTER_LINEAR)
	mprime = (256*mprime).astype('int32')
	cv2.imwrite("test2-guess.png", mprime)
	turnRightAngle(15)
	n = sweep(50)
	cv2.imwrite("test2.png", n)

	# Rotate Right
		

def mapTheWorld():
	N = 139//2
	mask = np.ones((N, 2*N, 3))
	world = np.zeros((1024, 1024, 3))
	grid = np.zeros((100, 100), 'int32')
	X = (511, 1023)	# Car position in world coordinates
	R = np.radians(90) # Car heading in world coordinates
	goalLocation = (511, 0)

	step = 0
	while step < 5:
		print("step {0} car {1} heading {2}".format(step, X, np.degrees(R)))
		m = sweep(72, True)
		# The scan area to the car is always in local coordinates
		# We will map them back into world coordinates
		# Note that the scale is the same so all we have to deal with 
		# is translation and rotation
		pts1 = np.float32([[N, 0], [N, N], [2*N, N]])
		pts2 = np.float32([
			[X[0] + N * np.cos(R),
			 X[1] - N * np.sin(R)],

			[X[0], X[1]],

			[X[0] + N * np.cos(R - np.pi/2),
			 X[1] - N * np.sin(R - np.pi/2)]])

		print(pts1)
		print(pts2)
		mat = cv2.getAffineTransform(pts1, pts2)
		dst = cv2.warpAffine(m.astype('float'), mat, world.shape[1::-1])
		mask_t = cv2.warpAffine(mask, mat, world.shape[1::-1])

		print(mask_t.shape)
		print(world.shape)
		print(dst.shape)

		world2 = world.copy()
		for c in range(3):
			world2[:,:,c] = (1 - mask_t[:,:,c])*world[:,:,c] + \
			mask_t[:,:,c]*dst[:,:,c]
		world = world2

		#draw a picture of car and current heading on 
		# pic for display
		#world2 = cv2.circle(world2, X, radius=3, color=(255, 255, 255), thickness=1)
		#cv2.imwrite("world_{0}.png".format(step), world2)

		# Try to take a forward step and remap
		fc.forward(10)
		time.sleep(0.5)
		fc.forward(0)
		X = (X[0], X[1] - 15)
		#X = (X[0] + 15, X[1] - 15)
		#R = R + np.pi / 3
		step = step + 1

def turnAngle(angle):
	if (angle > 0): turnLeftAngle(angle)
	else: turnRightAngle(-angle)

def turnLeftAngle(angle):
	# I told it to turn 15, and I think it turned 25
	# so I need to scale this by (15/25) = 3/5
	#2.78 by experimentation.
	duration = (angle /180.) * (1.32)
	fc.turn_left(10)
	time.sleep(duration)
	fc.turn_left(0)

def turnRightAngle(angle):
	#2.78 by experimentation.
	duration = (angle / 180.) * (1.98)
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

