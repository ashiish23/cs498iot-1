import picar_4wd as fc
import numpy as np
import heapq
import time

from astar import AStar, MinTurns

inc = 10
pow = 30
agL = 0.740
agR = 0.710
fcm = 0.025
	
start = (100,50)

def main():
	
	time.sleep(5)
	
	dist1 = getDist(inc)
	map1 = getMap(dist1)
	np.savetxt("maps/map1.csv", map1, fmt='%i', delimiter=",")	
	#map1 = np.loadtxt("maps/map1.csv", dtype="int", delimiter=",")	
	
	goal1 = (0,0)
	turns1, route1 = getRoute(map1, goal1)
	np.savetxt("maps/route1.csv", route1, fmt='%i', delimiter=",")
	
	naviCar(turns1)

	time.sleep(20)
	
	dist2 = getDist(inc)
	map2 = getMap(dist2)
	np.savetxt("maps/map2.csv", map2, fmt='%i', delimiter=",")	
	#map1 = np.loadtxt("maps/map1.csv", dtype="int", delimiter=",")	
	
	goal2 = (0,100)
	turns2, route2 = getRoute(map2, goal2)
	np.savetxt("maps/route2.csv", route2, fmt='%i', delimiter=",")
	
	naviCar(turns2)
	

def getDist(inc):
	disa = []
	a = -90
	while a <= 90:
		disa.append(fc.get_distance_at(a))
		#time.sleep(0.1)
		a += inc
	print("DistanceA: ", disa)
	
	disb = []
	b = 90
	while b >= -90:
		disb.append(fc.get_distance_at(b))
		#time.sleep(0.1)
		b -= inc
	print("DistanceB: ", disb)
	
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
			y=(h-1)-int(round(dist[i]*np.sin((i*inc)*np.pi/180)))
			x=int(w/2)+int(round(dist[i]*np.cos((i*inc)*np.pi/180)))
			#print([dist[i],-90+i*inc,y,x])
			
			if 0 <= y < h and 0 <= x < w:
				#print("print!")
				map[y][x] = 2
				
				# add padding
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
	mpn = ["" for i in range(len(map))]
	for j in range(len(map)):
		for i in map[j]:
			mpn[j] += str(int(i))
	
	#print("MPN: ", mpn)
	turns = MinTurns(start, goal, mpn).search()
	turns.append(goal)
	print("Turns: ", turns)
	
	cur = start
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
	
	for r in route:
		map[r[0]][r[1]] = 9
	
	return turns, map


def naviCar(turns):
	dir = []
	o = start
	d = 'F'

	for t in turns:
		
		dy = t[0] - o[0]
		dx = t[1] - o[1]
		
		if dy == 0 and dx > 0:
			dir.append('R')
			if d == 'B' or d == 'F':
				fc.turn_right(pow)
				time.sleep(agR)
			d = 'R'
			
		if dy == 0 and dx < 0:
			dir.append('L')
			if d == 'F' or d == 'F':	
				fc.turn_left(pow)
				time.sleep(agL)
			d = 'L'
			
		if dy > 0 and dx == 0:
			dir.append('B')
			if d == 'L':
				fc.turn_right(pow)
				time.sleep(agR)
			if d == 'R':	
				fc.turn_left(pow)
				time.sleep(agL)
			d = 'B'
			
		if dy < 0 and dx == 0:
			dir.append('F')
			if d == 'L':
				fc.turn_right(pow)
				time.sleep(agR)
			if d == 'R':	
				fc.turn_left(pow)
				time.sleep(agL)
			d = 'F'
		
		fc.stop()	
		fc.forward(pow)
		time.sleep(fcm * abs(dy+dx))
		
		o = t
	
	fc.stop()

	print("Direction: ", dir)


if __name__ == "__main__":
    main()
