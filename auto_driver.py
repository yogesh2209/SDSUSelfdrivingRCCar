from pivideostream import PiVideoStream
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from rc_car import *
from sensor import *
import time
import math
import struct
from PIL import Image
import numpy as np





class ObjectDetection(object):
    def __init__(self):
        self.red_light = False
        self.green_light = False

    
    def detect(self, image, gray, classifier):
        v = 0
        threshold = 150

    
        objects = classifier.detectMultiScale(gray,1.3,5)
        for (x,y,w,h) in objects:
            detected_object =  cv2.rectangle(image, (x+5, y+5), (x+w-5, y+h-5), (255, 255, 255), 2)
            v = y + h - 5
            # Region of interest - eyes detection 
            roi_color = image[y:y+h,x:x+w]
            roi_gray = gray[y:y+h,x:x+w]

            #stop sign
            if w / h == 1:
                cv2.putText(image, 'STOP', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                print('v is ',v)

            #traffic light
            else:
                roi = gray[y+10: y+h-10, x+10: x+w-10]

                #Apply gaussian blur through the image and find the brightest spot to determine
                #if light is on
                mask = cv2.GaussianBlur(roi, (25,25), 0)
                (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)

                #check to see if light is on
                if maxVal - minVal > threshold:
                    cv2.circle(roi, maxLoc, 5, (255, 0, 0), 2)

                    # Red light
                    if 1.0 / 8 * (height - 30) < maxLoc[1] < 4.0 / 8 * (height - 30):
                        cv2.putText(image, 'Red', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        self.red_light = True

                    # Green light
                    elif 5.5 / 8 * (height - 30) < maxLoc[1] < height - 30:
                        cv2.putText(image, 'Green', (x_pos + 5, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),2)
                        self.green_light = True


        return v
         
        
class DistanceToCamera(object):
    def __init__(self):
        # Params obtained from camera calibration
        self.alpha = 8.0 * math.pi / 180
        self.v0 = 119.865631
        self.ay = 332.262498

    def calculate(self, v, h, x_shift, image):
        self.alpha = 8.0 * math.pi / 180
        self.v0 = 119.865631
        self.ay = 332.262498
        """Calculates the distance to an object through a geometry model using monocular vision"""
        d = h / math.tan(self.alpha + math.atan((v - self.v0) / self.ay))

        if d > 0:
            cv2.putText(image, "%.1fcm" % d,
                        (image.shape[1] - x_shift, image.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return d

class VideoStreamHandler():
    def __init__(self):
        #Cascade classifiers
        self.stop_classifier = cv2.CascadeClassifier('./xml_files/stop_sign.xml')
        self.light_classifier = cv2.CascadeClassifier('./xml_files/traffic_light.xml')

        #Object detection instance
        self.obj_detection = ObjectDetection()

        #Distance to camera instance
        self.dist_to_camera = DistanceToCamera()
        self.h_stop = 14.5 - 10 # cm
        self.h_light = 14.5 - 10 # cm
        self.d_stop = 100.0
        self.d_light = 100.0


        self.minimum_distance = 140
        self.waiting_time_on_stoppage = 2
        self.reverse_time = 1.0
        self.forward_left_time = 0.5
        self.forward_again_time = 0.5
        self.forward_right_time = 0.5
        self.counter = 0

        #RC Car instance
        self.car = RC_Car()

        #Sensor instance
        self.sensor_data = SensorDistance()
        
    
    def handle(self):
        stop_sign_active = False
        stop_flag = False
        stop_start = 0
        stop_finish = 0
        stop_time = 0
        drive_time_after_stop = 0

        self.car.init()
        self.car.forward()
    
        thread_stream = PiVideoStream()
        thread_stream.start()
        time.sleep(1.0)

        
        # read the video frames one by one
        try:
            while True:
                # Original image
                image = thread_stream.read() 

                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                fname = "images/image_" + str(self.counter) + ".jpg"
                # Object detection
                v_stop = self.obj_detection.detect(image, gray, self.stop_classifier)
                v_light = self.obj_detection.detect(image, gray, self.light_classifier)

                # Show computed image
                cv2.imshow('Threaded Camera OpenCV Preview',image)
                


                #data1 = np.array(image, dtype='uint8')
                #print("length of numpy arr", len(data1))
            
                
                #cv2.imwrite(fname,image)
                #ab = cv2.imread("image.jpg")
                #img2 = Image.fromarray(ab, 'RGB')
                #img2.show()
                #self.counter = self.counter + 1
                
            
                #print("stop v", v_stop)
                #print("stop light", v_light)

                # Distance measurement
                if v_stop > 0 or v_light > 0:
                    d1 = self.dist_to_camera.calculate(v_stop, self.h_stop, 300, image)
                    d2 = self.dist_to_camera.calculate(v_light, self.h_light, 100, image)
                    self.d_stop = d1
                    self.d_light = d2


                print("distance from sensor", self.sensor_data.sensor_distance())

                # Check for stop conditions
                if self.sensor_data.sensor_distance() != 1000000 and self.sensor_data.sensor_distance() < 120:
                    # Front collision avoidance
                    self.car.stop()
                    print("Stopping, obstacle in front!")
                    print("Changing the direction")
                    #calling action for obstacle in front
                    self.obstacleInFrontAction()
                    

                elif 0.0 < self.d_stop < 80.0 and stop_sign_active:
                    print('Stop sign ahead. Stopping...')
                    
                    self.car.stop()

                    # Stop for 5 seconds
                    if stop_flag is False:
                        stop_start = cv2.getTickCount()
                        stop_flag = True

                    stop_finish = cv2.getTickFrequency()
                    stop_time = stop_finish - stop_start
                    print("Stop time:", stop_time)

                    # Waited 5 seconds, continue driving
                    if stop_time > 5:
                        stop_flag = False
                        stop_sign_active = False
                        print("Waited for 5 seconds")

                elif 0.0 < self.d_light < 80.0:
                    print("Light if")
                    if self.obj_detection.red_light:
                        print("Red light")
                        self.car.stop()
                    elif self.obj_detection.green_light:
                        print("Green light")
                        pass
                        
                    self.obj_detection.red_light = False
                    self.obj_detection.green_light = False
                    self.d_light = 80.0

                else:
                    print("else conditon running")
                    #self.car.init()
                    #self.car.forward()
                    self.d_stop = 80.0
                    stop_start = cv2.getTickCount()

                    if stop_sign_active is False:
                        drive_time_after_stop = (stop_start - stop_finish) / cv2.getTickFrequency()
                        if drive_time_after_stop > 5:
                            stop_sign_active = True

               

                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break

            # Close image window and thread
            cv2.destroyAllWindows()
            thread_stream.stop()

        finally:
            #self.car.stop()
            print("Connection closed on the server video thread!")


    def obstacleInFrontAction(self):
        self.car.init()
        self.car.reverse()
        time.sleep(0.50)
        self.car.stop()
        self.car.init()
        #wait for some seconds
        time.sleep(0.5)
        #reverse now for some time
        self.car.reverse()
        time.sleep(0.5)
        #stop the reversing and front left now for some time
        self.car.stop()

        self.car.init()
        #now turn right forward a little and then stop and make it forward
        self.car.turn_forward_right()
        time.sleep(1.5)
        self.car.stop()
       
        self.car.init()
        self.car.turn_forward_left()
        time.sleep(0.3)
        self.car.stop()
        self.car.init()
        #now turn right forward a little and then stop and make it forward
        self.car.forward()
        time.sleep(5.0)
        self.car.stop()
        
        
       

if __name__ == "__main__":
   videoHandler = VideoStreamHandler()
   videoHandler.handle()
    
