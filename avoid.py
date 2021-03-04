import picar_4wd as fc
import time

pow = 10
buf = 30
bak = 0.2

def main():
	time.sleep(5)

	while True:
		scan_list = fc.scan_step(buf)
		if not scan_list:
			continue
		tmp = scan_list[2:8]
		tmpL = scan_list[2:5]
		tmpR = scan_list[5:8]
		print(tmp)
		if tmp != [2,2,2,2,2,2]:
			print("Back Off")
			fc.backward(pow)
			time.sleep(bak)
			
			if sum(tmpL) > sum(tmpR):
				print("Turn Left")
				fc.turn_left(pow*2)			
			else:
				print("Turn Right")
				fc.turn_right(pow*2)
		
		else:
			fc.forward(pow)

if __name__ == "__main__":
	try: 
		main()
	finally: 
		fc.stop()
