from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import io, re, time, heapq, picamera

import numpy as np
import picar_4wd as fc

from astar import AStar, MinTurns
from PIL import Image
from tflite_runtime.interpreter import Interpreter

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 450
LABEL = "tflite/coco_labels.txt"
MODEL = "tflite/detect.tflite"
THRES = 0.40
OBJDT = 0.300

INC = 10
POW = 30
AGL = 0.740
AGR = 0.710
FCM = 0.025

START = (100,50)

def main():
	
	time.sleep(5)

	dist = getDist()
	map = getMap(dist)
	np.savetxt("maps/map8.csv", map, fmt='%i', delimiter=",")	

	goal = (0,0)
	turns, route = getRoute(map, goal)
	np.savetxt("maps/route8.csv", route, fmt='%i', delimiter=",")
	
	naviCar(turns)
	

def getDist():
	disa = []
	for a in range(-90, 100, INC):
		disa.append(fc.get_distance_at(a))
	#print("DistanceA: ", disa)
	
	disb = []
	for b in range(90, -100, -INC):
		disb.append(fc.get_distance_at(b))
	#print("DistanceB: ", disb)
	
	dist = []
	for c in range(len(disa)):
		dist.append(min(disa[c],disb[len(disa)-1-c]))
	print("Distances: ", dist)
	
	fc.servo.set_angle(0)
	
	return dist


def getMap(dist):
	w = 101
	h = 101
	p = 10
	b = 0
	pdy = p*2 + b + 1
	pdx = p*2 + 1
	
	map = np.zeros((h,w))
	for i in range(len(dist)):
		if dist[i] != -2:
			# convert polar to cartesian coordinates
			y=(h-1)-int(round(dist[i]*np.sin((i*INC)*np.pi/180)))
			x=int(w/2)+int(round(dist[i]*np.cos((i*INC)*np.pi/180)))
			#print([dist[i],-90+i*INC,y,x])
			
			# mark coordinates on map
			if 0 <= y < h and 0 <= x < w:
				map[y][x] = 2
				
				# add padding and mark them
				for m in range(pdy):
					if (y-p-b)+m < 0 or (y-p-b)+m > h-1:
						continue
					for n in range(pdx):
						if (x-p)+n < 0 or (x-p)+n > w-1:
							continue
						if map[(y-p-b)+m][(x-p)+n] != 2:
							map[(y-p-b)+m][(x-p)+n] = 1
	
	return map


def getRoute(map, goal):
	# convert marked map for A* algorithm
	mpn = ["" for i in range(len(map))]
	for j in range(len(map)):
		for i in map[j]:
			mpn[j] += str(int(i))
	
	# find best route with minimum direction changes
	turns = MinTurns(START, goal, mpn).search()
	turns.append(goal)
	print("Turns: ", turns)
	
	# create complete route with coordinates
	cur = START
	route = []
	for t in turns:
		y = t[0] - cur[0]
		x = t[1] - cur[1]
		if y > 0:
			for j in range(y):
				route.append((cur[0]+j,cur[1]))
		if y < 0:	
			for j in range(-y):
				route.append((cur[0]-j,cur[1]))
		if x > 0:
			for i in range(x):
				route.append((cur[0],cur[1]+i))
		if x < 0:	
			for i in range(-x):
				route.append((cur[0],cur[1]-i))
		cur = t
		
	route.append(goal)
	#print("Route: ", route)
	
	# mark route coordinates
	for r in route:
		map[r[0]][r[1]] = 9
	
	return turns, map


def naviCar(turns):
	
	labels = load_labels(LABEL)
	interpreter = Interpreter(MODEL, num_threads=3)
	interpreter.allocate_tensors()
	_, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']

	camera = picamera.PiCamera(
			resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=90)
	camera.rotation = 180
	camera.video_stabilization = True 
	
	dir = []
	o = START
	d = 'F'
	id = 0

	for t in turns:
		
		#print("==========>", t)
		
		dy = t[0] - o[0]
		dx = t[1] - o[1]
		
		if dy == 0 and dx > 0:
			dir.append('R')
			if d == 'B' or d == 'F':
				fc.turn_right(POW)
				time.sleep(AGR)
			d = 'R'
			
		if dy == 0 and dx < 0:
			dir.append('L')
			if d == 'F' or d == 'F':	
				fc.turn_left(POW)
				time.sleep(AGL)
			d = 'L'
			
		if dy > 0 and dx == 0:
			dir.append('B')
			if d == 'L':
				fc.turn_right(POW)
				time.sleep(AGR)
			if d == 'R':	
				fc.turn_left(POW)
				time.sleep(AGL)
			d = 'B'
			
		if dy < 0 and dx == 0:
			dir.append('F')
			if d == 'L':
				fc.turn_right(POW)
				time.sleep(AGR)
			if d == 'R':	
				fc.turn_left(POW)
				time.sleep(AGL)
			d = 'F'
			
		fc.stop()
		
		# object detect to start
		id += 1
		tStart = time.monotonic()
		image = getImage(input_height, input_width, camera, id)
		results = detect_objects(interpreter, image, THRES)
		tRef = getRespond(results, labels, 0) + (time.monotonic() - tStart)
		
		tBuffer = OBJDT
		tRemain = FCM * abs(dy+dx)
		tElapsed = 0
		tDrive = 0
		
		print('Reference Time: %.3f' % tRef)
		print('Initial Remained Time: %.3f' % tRemain)
		
		fc.forward(POW)
		
		while tRemain - max(tElapsed, tBuffer) > 0:
			tStart = time.monotonic()
			image = getImage(input_height, input_width, camera, 0)
			results = detect_objects(interpreter, image, THRES)
			tElapsed = (time.monotonic() - tStart) + getRespond(results, labels, 1)
			tDrive += tElapsed
			#tRemain -= max(tElapsed, tBuffer)
			tRemain -= tElapsed
			print('Loop Elapsed | Remain Time: %.3f | %.3f' % (tElapsed, tRemain))
		
		time.sleep(tRemain)
		print('Turn Drive Time: %.3f' % (tDrive+tRemain))
		
		o = t
	
	fc.stop()
	print("Direction: ", dir)
	camera.close()


def getRespond(results, labels, action):

	tCheck = time.monotonic()
	#tUsed = 0
	
	stopsign = 0
	bicycle = 0
	
	objects = []
	for obj in results:
		objects.append(labels[obj['class_id']])
		if labels[obj['class_id']] == 'stop sign':
			stopsign = 1

		if labels[obj['class_id']] == 'bicycle':
			bicycle = 1
		
	if action == 1:
		if stopsign == 1:
			print("Stop Here!!")
			time.sleep(0.1)
			fc.stop()
			#tUsed += time.monotonic() - tCheck 
			time.sleep(2)
			tCheck = time.monotonic()
			fc.forward(POW)
			stopsign = 0
			print("Objects: ", objects)	
			
		if bicycle == 1:
			print("Slow Down!!")
			#tUsed += time.monotonic() - tCheck 
			time.sleep(0.1)
			fc.stop()
			time.sleep(0.2)
			fc.forward(POW/3)
			time.sleep(1)
			fc.stop()
			time.sleep(0.2)
			tCheck = time.monotonic() - 1
			fc.forward(POW)
			bicycle = 0
			print("Objects: ", objects)		
	
	#tUsed += time.monotonic() - tCheck
	return time.monotonic() - tCheck


def getImage(input_height, input_width, camera, id):	
	
	stream = io.BytesIO()
	
	if id > 0:
		camera.capture('imgs/capture%d.jpg' % id, use_video_port=True)
	
	camera.capture(stream, format='jpeg', use_video_port=True)
	
	stream.seek(0)
	image = Image.open(stream).convert('RGB').resize(
			(input_width, input_height), Image.ANTIALIAS)
	
	stream.truncate()
	
	return image
	

def load_labels(path):
	"""Loads the labels file. Supports files with or without index numbers."""
	with open(path, 'r', encoding='utf-8') as f:
		lines = f.readlines()
		labels = {}
		for row_number, content in enumerate(lines):
			pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
			if len(pair) == 2 and pair[0].strip().isdigit():
				labels[int(pair[0])] = pair[1].strip()
			else:
				labels[row_number] = pair[0].strip()
	return labels


def set_input_tensor(interpreter, image):
	"""Sets the input tensor."""
	tensor_index = interpreter.get_input_details()[0]['index']
	input_tensor = interpreter.tensor(tensor_index)()[0]
	input_tensor[:, :] = image


def get_output_tensor(interpreter, index):
	"""Returns the output tensor at the given index."""
	output_details = interpreter.get_output_details()[index]
	tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
	return tensor


def detect_objects(interpreter, image, threshold):
	"""Returns a list of detection results, each a dictionary of object info."""
	set_input_tensor(interpreter, image)
	interpreter.invoke()

	# Get all output details
	boxes = get_output_tensor(interpreter, 0)
	classes = get_output_tensor(interpreter, 1)
	scores = get_output_tensor(interpreter, 2)
	count = int(get_output_tensor(interpreter, 3))

	results = []
	for i in range(count):
		if scores[i] >= threshold:
			result = {
				'bounding_box': boxes[i],
				'class_id': classes[i],
				'score': scores[i]
			}
			results.append(result)
	return results
	
	
if __name__ == "__main__":
    main()
