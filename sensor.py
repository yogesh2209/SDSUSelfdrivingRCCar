import RPi.GPIO as gpio
import time
import sys
import time

class SensorDistance():
    def __init__self():
        self.init()

    def init(self):
        #set GPIO Pins
        gpio.setmode(gpio.BOARD)
        gpio.setup(16, gpio.OUT)
        gpio.setup(18, gpio.IN)
        gpio.output(16, True)
        time.sleep(0.0000001)
        gpio.output(16, False)
        

    def sensor_distance(self, measure='cm'):
        try:
            self.init()
            while gpio.input(18) == 0:
                nosig = time.time()

            while gpio.input(18) == 1:
                sig = time.time()

            tl = sig - nosig

            if measure == 'cm':
                distance = tl / 0.000058
            elif measure == 'in':
                distance = tl / 0.000148
            else:
                print('improper choice of measurement: in or cm')
                distance = None

            gpio.cleanup()
            return distance
        except:
            #fake value of distance
            distance = 1000000
            gpio.cleanup()
            return distance
