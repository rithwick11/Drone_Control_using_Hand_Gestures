from __future__ import print_function
import cv2
import time
import math
import threading
import numpy as np
import hand_detect as hd
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

wCam, hCam = 640, 480

cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)
pTime = 0
detector = hd.handDetector(detectionCon=0.7, maxHands=1)
area = 0

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect("127.0.0.1:14550", wait_ready=True)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    
    
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
                             
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(7.5)

print("Airspeed set to 2")
vehicle.airspeed = 2
'''
while True:
	print(" Altitude: ", vehicle.location.global_relative_frame.alt)
	print(" Velocity: ", vehicle.velocity)
	print(" GSpeed: ", vehicle.groundspeed)
time.sleep(20)
'''
while True:
    success, img = cap.read()

    # Find Hand
    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img, draw=True)
    if len(lmList) != 0:

        # Filter based on size
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) // 100
        # print(area)
        if 250 < area < 1000:
            length_ti, img, lineInfo = detector.findDistance(4, 8, img)
            length_tm, img, lineInfo = detector.findDistance(4, 12, img)
            length_tr, img, lineInfo = detector.findDistance(4, 16, img)
            length_tp, img, lineInfo = detector.findDistance(4, 20, img)
            length_timcp, img, lineInfo = detector.findDistance(4, 5, img)
            length_tmmcp, img, lineInfo = detector.findDistance(4, 9, img)
            length_trmcp, img, lineInfo = detector.findDistance(4, 13, img)
            length_tpmcp, img, lineInfo = detector.findDistance(4, 17, img)
            #print(length_ti)
            #print(length_tm)
            #print(length_tr)
            #print(length_tp)
			#length_im, img, lineInfo = detector.findDistance(8, 12, img)
			#length_mr, img, lineInfo = detector.findDistance(12, 16, img)
			#length_rp, img, lineInfo = detector.findDistance(16, 20, img)
            #print(length)
            # move drone according to gestures:
            '''
			# Set up velocity mappings
			# velocity_x > 0 => fly North
			# velocity_x < 0 => fly South
			# velocity_y > 0 => fly East
			# velocity_y < 0 => fly West
			# velocity_z < 0 => ascend
			# velocity_z > 0 => descend
			SOUTH=-2
			UP=-0.5   #NOTE: up is negative!

			#Fly south and up.
			send_ned_velocity(SOUTH,0,UP,DURATION)
			'''
            if (length_ti <= 40):
                send_ned_velocity(2,0,0,1)
                print('Drone moving Forward')
            elif (length_tm <= 40):
                send_ned_velocity(-2,0,0,1)
                print('Drone moving Backward')
            elif (length_tr <= 40):
                send_ned_velocity(0,2,0,1)
                print('Drone moving Right')
            elif (length_tp <= 40):
                send_ned_velocity(0,-2,0,1)
                print('Drone moving Left')
            elif (length_timcp <= 40):
                send_ned_velocity(0,0,-2,1)
                print('Drone moving Up')
            elif (length_tmmcp <= 40):
                send_ned_velocity(0,0,2,1)
                print('Drone moving Down')
            elif (length_trmcp <= 40):
                vehicle.mode = VehicleMode("RTL")
                print('Drone returns to launch')

    # Frame rate
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (40, 50), cv2.FONT_HERSHEY_COMPLEX,
                1, (255, 0, 0), 3)

    cv2.imshow("Img", img)
    cv2.waitKey(1)

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
