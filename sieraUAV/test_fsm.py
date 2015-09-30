#!/usr/bin/python
'''
	TEST FSM
'''

#!/usr/bin/python
import time
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import sys

sys.path.append('./')
from fsm.fsm_test_fl import *
from testalgo import test_th
from misc.th_misc import *

#Global init
api=None
vehicle=None


def init_api():
    global api
    global vehicle
    api= local_connect()
    vehicle = api.get_vehicles()[0]


#Main

#Create thread
threadCV = myThread(1, "threadCV",test_th)

#Start thread
threadCV.start()

init_api()

fsm=fsm_test(api, vehicle, (48.789222, 2.028945 ),7)


print "Waiting for video"
time.sleep(3)

while not api.exit and threadCV.is_alive():
	#Update var
	fsm.rt_update_var()

	#Run FSM
	fsm.rt_fsm_mng()

	time.sleep(0.1)


threadCV.join()