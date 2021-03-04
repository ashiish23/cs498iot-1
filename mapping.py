import picar_4wd as fc
import time
import numpy as np

def main():
	dist = []
	inc = 10
	
	disa = []
	a = -90
	while a <= 90:
		disa.append(fc.get_distance_at(a))
		a += inc
	print(disa)
	
	disb = []
	b = 90
	while b >= -90:
		disb.append(fc.get_distance_at(b))
		b -= inc
	print(disb)
	
	for c in range(len(disa)):
		dist.append(min(disa[c],disb[len(disa)-1-c]))
	print(dist)
	
	fc.servo.set_angle(0)

	np.savetxt("maps/map.csv", getMap(dist,inc), fmt='%i', delimiter=",")
	

def getMap(dist,inc):
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
			print([dist[i],-90+i*inc,y,x])
			
			if 0 <= y < h and 0 <= x < w:
				print("print!")
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
	

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
