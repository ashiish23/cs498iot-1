import picar_4wd as fc
import time

pow = 50
ang = 0

def main():
	time.sleep(5)
	fc.forward(pow)
	while True:
		dist = fc.get_distance_at(ang)
		print(dist)
		if dist != -2:
			if dist <= 60:
				fc.forward(pow/5)		
			if dist <= 20:
				fc.stop()
				break;
	fc.stop()

if __name__ == "__main__":
	main()