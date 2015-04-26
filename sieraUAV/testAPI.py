#
# This is a small example of the python drone API
# Usage:
# * mavproxy.py
# * module load api
# * api start small-demo.py
#
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time

def mavrx_debug_handler(message):
    """A demo of receiving raw mavlink messages"""
    print "Received", message

# First get an instance of the API endpoint
api = local_connect()
# get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
v = api.get_vehicles()[0]


Fixtype={
	 0 :"No",
	 1 :"NoFix",
	 2 :"2D",
	 3 :"3D",
	 4 :"DGPS",
	 5 :"RTK"
 }

def initHexa():

	# Disarm
	print "Disarming..."
	v.armed = False
	v.flush()

	# Wait 3d fix 
	while Fixtype [v.gps_0.fix_type] != "3D":
		print "Wait 3d fix, current fix GPS is : %s" % Fixtype [v.gps_0.fix_type]
		time.sleep(1)

	#Check altimeter

	# Check waypoint before takeof

	cmds = v.commands
	cmds.download()
	cmds.wait_valid()
	print "Home WP: %s" % cmds[0]
	print "Current dest: %s" % cmds.next
	time.sleep(5)


def Mission():

	# Change MODE --> AUTO
	vehicle.mode    = VehicleMode("AUTO")

	#ARME HEXA
	print "Arming..."
	v.armed = True
	v.flush()

	while not vehicle.armed and not api.exit:
		print "Waiting for arming..."
		time.sleep(1)



def main():

	#INIT HEXA
	initHexa()

	#TIME BEFORE MISSION LAUNCH
	print "lancement de la mission dans 10 seconde"
	i=10
	while i>0:
		print ("Mission dans: %d" % i)
		time.sleep(1)
		i-=1

	print "lancement de la mission"

	# EXE MISSION
	Mission()

###########################
# Execute main function
###########################

main()     




	