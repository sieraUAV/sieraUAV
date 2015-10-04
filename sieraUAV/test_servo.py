#!/usr/bin/python
import time
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil

import cv2
import numpy as np
from math import pi, cos, sin
import sys

sys.path.append('./')
from video_th import *

api             = 0
vehicle         = 0

def init_api():
    global api
    global vehicle
    api= local_connect()
    vehicle = api.get_vehicles()[0]
    vehicle.set_mavlink_callback(mavrx_debug_handler)


def mavrx_debug_handler(message):
    print "Received", message

def arm_and_takeoff():
    global api
    global vehicle

    """Dangerous: Arm and takeoff vehicle - use only in simulation"""
    # NEVER DO THIS WITH A REAL VEHICLE - it is turning off all flight safety checks
    # but fine for experimenting in the simulator.

    print "Waiting for GPS..."
    while vehicle.gps_0.fix_type < 3:
        # gps_0.fix_type:
        # 0-1: no fix
        # 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
        # check https://pixhawk.ethz.ch/mavlink/#GPS_RAW_INT
        time.sleep(1)

    print "Waiting for location..."
    while vehicle.location.alt == 0.0:
        time.sleep(1)

    print "Arming..."
    vehicle.mode    = VehicleMode("STABILIZE")
    #Securite non inibe
    #vehicle.parameters["ARMING_CHECK"] = 0
    vehicle.armed   = True
    vehicle.flush()

    print "Waiting for arming cycle completes..."
    while not vehicle.armed and not api.exit:
        time.sleep(1)        

    print("Setting AUTO mode...")
    vehicle.mode = VehicleMode("AUTO")
    vehicle.flush()

    time.sleep(1)

    print "Taking off!"
    #Takeoff mode auto
    #vehicle.commands.takeoff(5) # Take off to 5m height
    vehicle.flush()
    time.sleep(10)

# send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
def send_nav_velocity(velocity_x, velocity_y, velocity_z):
    global api
    global vehicle

    # create the SET_POSITION_TARGET_LOCAL_NED command
    # Check https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED
    # for info on the type_mask (0=enable, 1=ignore).
    # Accelerations and yaw are ignored in GCS_Mavlink.pde at the
    # time of writing.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink.pde)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink.pde) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_servo( servo, stategates):
    global api
    global vehicle

    # create the SET_POSITION_TARGET_LOCAL_NED command
    # Check https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED
    # for info on the type_mask (0=enable, 1=ignore).
    # Accelerations and yaw are ignored in GCS_Mavlink.pde at the
    # time of writing.

    msg=None

    st_open2=1050
    st_close=1500
    st_open1=1950

    #state
    if stategates==1:
        pwm=st_open1
    elif stategates==0:
        pwm=st_close
    elif stategates==2:
        pwm=st_open2
    else:
        pwm=st_close

    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,   #command
        0,  #confirmation
        servo,  #param 1
        pwm,    #param 2
        0, 0, 0, 0, 0)   

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


# condition_yaw - send condition_yaw mavlink command to vehicle so it points at specified heading (in degrees)
def condition_yaw(heading):
    global api
    global vehicle
    # create the CONDITION_YAW command
    msg = vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
            0,     # sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
            2, # current - set to 2 to make it a guided command
            0, # auto continue
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

#yaw en rad repere relatif --> absolu
def chg_base_abs(xr, yr, yaw):

    xa= xr*cos(yaw) + yr*sin(yaw)
    ya= -xr*sin(yaw) + yr*cos(yaw)

    return (xa,ya)


########################
#GLOBAL VAR
########################

# vx > 0 => fly North
# vx < 0 => fly South

NORTH=2
SOUTH=-2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST=2
WEST=-2

# Note for vz: 
# vz < 0 => ascend
# vz > 0 => descend
UP=-0.5
DOWN=0.5

DURATION=60

#ASSER FUNCTION FOR TRACKING
counter_lost_obj=0;
def tracking():
    global counter_lost_obj
    
    #init local var 
    st_track=False
    Queue_obj=0

    #CORRECTOR VAR
    p_Corec= 0.01
    sat_Corec=2

    if not Q_RX.empty():
        #GET QUEUE INFO
        Queue_obj= Q_RX.get()

        if Queue_obj[0]=='DETECT':
            counter_lost_obj=0
            #BEGIN TRACKING
            (dstx,dsty)= Queue_obj [1]
            relative_yaw_tg= Queue_obj [2]

            #Compute speed
            Vx_con= dstx*p_Corec
            Vy_con= dsty*p_Corec

            #If we are near the arrow
            if abs(dstx)< 100 and abs(dsty)<100 :
                #Compute cap target
                cap_con= relative_yaw_tg  + vehicle.attitude.yaw * 180/pi
                if cap_con>360:
                    cap_con-= 360
                elif cap_con<0:
                    cap_con+= 360
            else:
                cap_con=0

            #Saturation
            if Vx_con> sat_Corec:
                Vx_con= sat_Corec
            if Vy_con> sat_Corec:
                Vy_con= sat_Corec


            print dstx, dsty, cap_con

            #Passage en repere absolu
            (Vx_con_a, Vy_con_a)= chg_base_abs(Vx_con, Vy_con, vehicle.attitude.yaw)
            print Vx_con_a, Vy_con_a

            condition_yaw(cap_con)
            send_nav_velocity(Vy_con_a, Vx_con_a, 0)
            st_track= True

        else:
            if (counter_lost_obj>6):
                condition_yaw(0)
                send_nav_velocity(0,0,0)
                counter_lost_obj=7
            else:
                counter_lost_obj+=1

    return st_track


#EXE MAIN
# start video thread 
# try:
#    thread.start_new_thread( videoThread )
# except:
#    print "Error: unable to start thread"


# #Create thread
# threadCV = myThread(1, "threadCV",videoThread)

# #start thread
# threadCV.start()
#Conect to api drone mavproxy
init_api()

# #Execute mission
# mission()

# #Attente fin thread video
# while threadCV.is_alive():
#     print vehicle.mode
#     time.sleep(0.1)

# threadCV.join()


while 1:
    print "servo 1"
    send_servo(10,1)
    time.sleep(2)
    print "servo 0"
    send_servo(10,0)
    time.sleep(2)
    print "servo 2"
    send_servo(10,2)
    time.sleep(2)