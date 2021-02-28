import picar_4wd as fc
import time

def move25():
    speed4 = fc.Speed(25)
    speed4.start()
    fc.backward(100)
    x = 0
    for i in range(10):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        print("%smm/s"%speed)
    print("%smm"%x)
    speed4.deinit()
    fc.stop()

move25()

