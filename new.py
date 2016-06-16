from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import numpy as np
import cv2
import argparse
import math

a_center = (1280, 800)
found = False
target_pitch = 0
target_heading = 0
areas = []
largest = 0
  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

def wrap_PI(angle): #from balloon_finder
    if (angle > math.pi):
        return (angle - (math.pi * 2.0))
    if (angle < -math.pi):
        return (angle + (math.pi * 2.0))
    return angle

def is_centered(center):
    #if(a_center.x - center.x > 5 or a_center.y - center.y > 5):
    if math.fabs(wrap_PI(vehicle.attitude.yaw - target_heading)) < math.radians(5):
        return True
    else:
        return False

def send_center(cnt):
    msg = vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     cnt, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
    
    vehicle.send_mavlink(msg)
    print 'Vehicle is Aligning'
    vehicle.flush()

def send_rise():
    msg = vehicle.message_factory.mission_item_encode(0,0,0, 
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 2, 0, 0,0,0,0,0,0,.5)                                                                                                                                                    
    vehicle.send_mavlink(msg)
    print 'Vehicle is Rising'
    vehicle.flush()

def align(x, y):#, target_x, target_y): #adapted from balloon finder
    pt = (x, y)
    attitude = vehicle.attitude

    if found:
        target_pitch, target_heading = pixels_to_direction(x, y, attitude.roll, attitude.pitch, attitude.yaw) #target_x, target_y
        #target_heading = math.radians(target_heading)
        #target_pitch = math.radians(target_pitch)
        if not is_centered(center):
            send_center(target_heading)
            print 'needs to be centered'
        else:
            return True
            print 'is centered'

    else:
        send_rise()
        sleep(5)
        send_center(target_heading)
        print 'keep looking' 
        
    #vehicle.gimbal.target_location(vehicle.home_location) #use only in SITL

def rotate_xy(x, y, angle): #from red balloon tracker project
    cos_ang = math.cos(angle)
    sin_ang = math.sin(angle)
    x_centered = x - cap.get(3)
    y_centered = y - cap.get(4)
    x_rotated = x * cos_ang - y * sin_ang
    y_rotated = x * sin_ang + y * cos_ang
    return x_rotated, y_rotated

 #from red balloon tracker project
#def findArea(contour):
 #   a = cv2.contourArea(contour)
  #  areas.append(a)
   # print 'appended'
    #if(len(areas)  >= len(contours)):
     #   print 'sizes are equal - EXIT LOOP!'
       # break

def pixels_to_direction(pixels_x, pixels_y, vehicle_roll, vehicle_pitch, vehicle_yaw):
        #horiz = 70
        #vert = 40
        x_rotated, y_rotated = rotate_xy(pixels_x - cap.get(3)/2, pixels_y - cap.get(4)/2, vehicle_roll) #cap.get(3) is width, 4 = height
        pitch_pixel_shift = int(math.degrees(vehicle_pitch) / 40 * cap.get(4))
        pitch_dir = (-y_rotated + pitch_pixel_shift) / float(cap.get(4)) * 40
        # calculate yaw shift in degrees
        yaw_dir = x_rotated / float(cap.get(3)) * 70 + math.degrees(vehicle_yaw)
        # return vertical angle to target and heading
        return pitch_dir, yaw_dir


#def main():
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap.open()

while(True):
     _, frame = cap.read() # _, 
     #frame = cap.set(3, 640)
     #frame = cap.set(4, 480)
     
     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     #cv2.imshow('mine', hsv)

     lower_blue = np.array([110,100,100])
     upper_blue = np.array([130,255,255])

     mask = cv2.inRange(hsv, lower_blue, upper_blue)
     #res = cv2.bitwise_and(frame,frame, mask= mask)

     erosion1 = cv2.erode(mask, np.ones((5,5),np.uint8), iterations = 1)
     dilate1 = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations = 1)
     erosion2 = cv2.erode(mask, np.ones((5,5),np.uint8), iterations = 1)
     dilate2 = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations = 1)
     cv2.imshow('mask', mask)

     cv2.blur(mask, (5,5)) #3,3
     _, thresh = cv2.threshold(mask, 15, 255, cv2.THRESH_BINARY)
     _, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

     for i, c in enumerate(contours):
        if len(areas) > len(contours): #if more areas than contours, refill
                                       #the areas array
            areas = []
           # findArea(c)
            a = cv2.contourArea(c)
            areas.append(a)
            #print 'appended'
            if(len(areas)  >= len(contours)):
                 print 'sizes are equal - EXIT LOOP!'
                 break
        else:
            #findArea(c)
            a = cv2.contourArea(c) #keep adding areas of found contours
            areas.append(a)
            #print 'appended'
            if(len(areas)  >= len(contours)):
                 print 'sizes are equal - EXIT LOOP!'
                 break


     #largest = cv2.contourArea(contours[i])
     print 'I added the contours'
     print 'size of contours is ' + str(len(contours))
     print 'size of areas is ' + str(len(areas))

     if(len(areas) > 0 and len(areas) == len(contours)):
       found = True
       cmax = 0 #areas[0]
       #for con in areas:
       for i, con in enumerate(areas):
          if cmax < con and i < len(areas) and con > 0:
            cmax = con
            largest = contours[i]
       #for i in range(len(areas)):
        #   if cmax < areas[i] and i < len(areas):
         #      largest = contours[i]
       print 'I found the largest contour - size of: ' + str(largest)

       final = cv2.drawContours(mask, contours, -1, (255,255,255), 3)

       #epsilon = .1 * cv2.arcLength(cmax, True);
       #cv2.approxPolyDP(cmax, epsilon, True) #cmax
       #largest = contours[cmax]
       if largest.any() != 0:
        (x,y), radius = cv2.minEnclosingCircle(largest)
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        cv2.circle(mask, center, radius, (0, 255, 0), 2)
 
        cv2.imshow('frame', frame)

        align(x, y)
        print 'aligned'
     
     if(cv2.waitKey(30) >= 0):
        break

cap.release()

         


    


    

    
    
    
