import serial
from nanpy import(ArduinoApi,SerialManager)  # Motor unit control for the robot 
from __future__ import print_function
import numpy as np
import cv2 as cv   # Import image processing function 
import math  # Math function  for the calculation 
from video import create_capture
from common import clock, draw_str  # Draw the string on the image processing 
import microgear.client as microgear   # Inter net of thing 
import time
import logging

appid = "Kornbot"
gearkey = "wBHqON1EtNqlTzu"
gearsecret =  "nt0utSlDrPEOiYOFFfHYJDbEw"


microgear.create(gearkey,gearsecret,appid,{'debugmode': True}) 
sensor1 = 0
sensor2 = 0
# Path calculation function for the imageprocessing function and control combine
deg = 0   # Setting the degree calculate path 
centre_x = 0 
centre_y = 0  
try:
   connection = SerialManager()
   motorunit = ArduinoApi(connection=connection) #Connection astrablished 

except:
    print("Motor unit control ")
#try:
 #  sensor_msg = serial.Serial("/dev/ttyUSB0",115200)
#except:
#   print("Sensor read message error please check the sensor unit")

   # Backward function for the robot to move back and detect obstable 
def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30),
                                     flags=cv.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects
def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv.rectangle(img, (x1, y1), (x2, y2), color, 2)

def Backward_active(sensor1,SpeedStart,SpeedEnd,timechange):
      if int(sensor1) >= 50:
             motorunit.analogWrite(6,0)
             motorunit.analogWrite(10,0)
             motorunit.analogWrite(4,0)
             motorunit.analogWrite(3,0)
             motorunit.analogWrite(9,SpeedEnd)
             motorunit.analogWrite(11,SpeedEnd)
             motorunit.analogWrite(2,SpeedEnd)
             motorunit.analogWrite(5,SpeedEnd)
      else:
             #Backward part
             motorunit.analogWrite(6,SpeedStart)     # Roughly 150    
             motorunit.analogWrite(10,SpeedStart)
             motorunit.analogWrite(4,SpeedStart)
             motorunit.analogWrite(3,SpeedStart)
             time.sleep(timechange) # time sleep speed change 0.05
             motorunit.analogWrite(6,SpeedEnd)           # Roughly 127
             motorunit.analogWrite(10,SpeedEnd)
             motorunit.analogWrite(4,SpeedEnd)
             motorunit.analogWrite(3,SpeedEnd)
             #Forward function  
             motorunit.analogWrite(9,0)
             motorunit.analogWrite(11,0)
             motorunit.analogWrite(2,0)
             motorunit.analogWrite(5,0)
    # Forward and detect the obstacle in front 
def Foward_active(sensor1,sensor2,SpeedStart,SpeedEnd,timechange): 
       if int(sensor2) >= 30 or int(sensor2) <= 50:
             motorunit.analogWrite(6,0)
             motorunit.analogWrite(10,0)
             motorunit.analogWrite(4,0)
             motorunit.analogWrite(3,0)
             motorunit.analogWrite(9,SpeedStart)
             motorunit.analogWrite(11,SpeedStart)
             motorunit.analogWrite(2,SpeedStart)
             motorunit.analogWrite(5,SpeedStart)
       if int(sensor1) >= 50:  
                  motorunit.analogWrite(6,0)
                  motorunit.analogWrite(10,0)
                  motorunit.analogWrite(4,0)
                  motorunit.analogWrite(3,0)
                  motorunit.analogWrite(9,SpeedEnd)
                  motorunit.analogWrite(11,SpeedEnd)
                  motorunit.analogWrite(2,SpeedEnd)
                  motorunit.analogWrite(5,SpeedEnd)   
       if int(sensor2) >= 20 or int(sensor2) <= 29:  # Mapping the value function for the Ultrasonics sensor  
             #Backward part
             #Backward part
             motorunit.analogWrite(6,SpeedStart)
             motorunit.analogWrite(10,SpeedStart)
             motorunit.analogWrite(4,SpeedStart)
             motorunit.analogWrite(3,SpeedStart)
             motorunit.analogWrite(9,0)
             motorunit.analogWrite(11,0)
             motorunit.analogWrite(2,0)
             motorunit.analogWrite(5,0)
             time.sleep(timechange)
             motorunit.analogWrite(6,SpeedEnd)
             motorunit.analogWrite(10,SpeedEnd)
             motorunit.analogWrite(4,SpeedEnd)
             motorunit.analogWrite(3,SpeedEnd)
             motorunit.analogWrite(9,0)
             motorunit.analogWrite(11,0)
             motorunit.analogWrite(2,0)
             motorunit.analogWrite(5,0)
def connection():
    	logging.info("Now I am connected with netpie")

def subscription(topic,message):
	logging.info(topic+" "+message)

def disconnect():
    
    logging.debug("disconnect is work")


microgear.setalias("VisualStudio")

microgear.on_connect = connection

microgear.on_message = subscription

microgear.on_disconnect = disconnect

microgear.subscribe("/Topic")

microgear.connect(False)
if __name__ == '__main__':
    import sys, getopt
    print(__doc__)

    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try:
        video_src = video_src[0]
    except:
        video_src = 0
    args = dict(args)
    cascade_fn = args.get('--cascade', "../../data/haarcascades/haarcascade_frontalface_alt.xml")
    nested_fn  = args.get('--nested-cascade', "../../data/haarcascades/haarcascade_eye.xml")

    cascade = cv.CascadeClassifier(cascade_fn)
    nested = cv.CascadeClassifier(nested_fn)

    cam = create_capture(video_src, fallback='synth:bg=../data/lena.jpg:noise=0.05')
    while True:
          sensor1 = motorunit.analogRead(0)#Sensor 1 functioning for the Back  infrared 
          sensor2 = motorunit.analogRead(1)#Sensor 2 functioning for the front ultrasonics
          ret, img = cam.read()
          gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
          gray = cv.equalizeHist(gray)
          t = clock()
          rects = detect(gray, cascade)
          vis = img.copy()
          draw_rects(vis, rects, (0, 255, 0))
          if not nested.empty():
                for x1, y1, x2, y2 in rects:   
                    print("Back sensor detection:")
                    print(sensor1)
                    print("Fron sensor distance :")
                    print(sensor2)
                    Foward_active(sensor1,sensor2,150,127,0.05) # Forward  
                    print("Facedetcted")
                    print(int(x1),int(y1))
                    print(int(x2),int(y2))
                    centre_x = int(x1) + 340 
                    centre_y = int(y1) + 240 
                    deg = 2*math.degrees(math.acos( (int(y1)+240)/(math.hypot(centre_x,centre_y)))) 
                    print("Degree:")
                    print(deg)   
                    microgear.chat("VisualStudio",deg,sensor1,sensor2,centre_x,centre_y)
                    roi = gray[y1:y2, x1:x2]
                    vis_roi = vis[y1:y2, x1:x2]
                    subrects = detect(roi.copy(), nested)
                    draw_rects(vis_roi, subrects, (255, 0, 0))
          dt = clock() - t

          draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
          cv.imshow('facedetect', vis)
          if cv.waitKey(5) == 27:
            break
    cv.destroyAllWindows()      
         