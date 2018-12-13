import RPi.GPIO as gpio
import time
import Tkinter as tk
import sys
import time
from sensor import *

class RC_Car():
    def __init__(self):
        self.init()
        
    def init(self):
        gpio.setmode(gpio.BOARD)
        gpio.setup(7, gpio.OUT)
        gpio.setup(11, gpio.OUT)
        gpio.setup(13, gpio.OUT)
        gpio.setup(15, gpio.OUT)
        gpio.setup(12, gpio.OUT)
        gpio.setup(22, gpio.OUT)
        gpio.output(13, True)
        gpio.output(11, True)
        gpio.output(15, True)
        gpio.output(12, True)   
        gpio.setwarnings(False)

    def forward(self):
        gpio.output(22, True)
        gpio.output(13, False)
        gpio.output(11, True)

    def reverse(self):
        gpio.output(22, True)
        gpio.output(13, True)
        gpio.output(11, False)

    def turn_forward_left(self):
        gpio.output(22, True)
        gpio.output(13, False)
        gpio.output(11, True)
        gpio.output(7, True)
        gpio.output(15, False)
        gpio.output(12, True)

    def turn_forward_right(self):
        gpio.output(22, True)
        gpio.output(13, False)
        gpio.output(11, True)
        gpio.output(7, True)
        gpio.output(15, True)
        gpio.output(12, False)

    def stop(self):
        gpio.cleanup()    
        




    

'''
#Used for keyboard inputs
def key_input(event):
    init()
    print 'Key:', event.char
    key_press = event.char
    sleep_time = 0.030

    if key_press.lower() == 'w':
        forward(sleep_time)
    elif key_press.lower() == 's':
        reverse(sleep_time)
    elif key_press.lower() == 'a':
        turn_left(sleep_time)
    elif key_press.lower() == 'd':
        turn_right(sleep_time)
    elif key_press.lower() =='q':
        pivot_left(sleep_time)
    elif key_press.lower() == 'e':
        pivot_right(sleep_time)

root = tk.Tk()
root.bind('<KeyPress>', key_input)
root.mainloop()     


if __name__ == '__main__':   
    try:
        while True:
           check_distance(minimum_distance)
 	   time.sleep(0.0005)		

    except KeyboardInterrupt:
        print("Measurement stopped by User")
        gpio.cleanup()google.    
'''
