import picar_4wd as fc
import time

pow = 10

def main():
	time.sleep(5)
	
	fc.forward(pow)
	time.sleep(3)
	
	fc.backward(pow)
	time.sleep(2)
	
	fc.turn_left(pow)
	time.sleep(2)
	
	fc.turn_right(pow)
	time.sleep(2)
	
	fc.stop()

if __name__ == '__main__':
	main()
