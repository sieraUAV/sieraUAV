#!/usr/bin/python
"""
@package This package to manage flight management FSM
"""
from math import pi, cos, sin
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
from pymavlink import mavextra
from misc.th_misc import Q_TX, Q_RX
from video_algo.algo_manage import ALGOS, STATUS_ALG
from video_algo.arrow import arrow_info
from misc.misc import *
from misc.geo_misc import chg_base_abs, get_dst_2WP
import time


GPS_FX=enum('KO', 'NO', 'FX_2D', 'FX_3D', 'DGPS', 'RTK')

FSM_ST=enum('INIT','TAKEOFF','NAV','ARROW_TRK', 'ARROW_ALGN', 'SQUR_TRK', 'LAND', 'ERROR')

class fsm_test:
	def __init__(self, api, vehicle, pt_gps_start, tg_alt=10):
		#Droneapi object
		self.api=api
		self.vehicle=vehicle

		#Global info
		self.tg_alt=tg_alt				#metres
		self.pt_gps_start=pt_gps_start	#(lat,lon)

		#FSM STATES var
		self.state=FSM_ST.INIT

		#Connection state
		self.st_exit=False

		#GPS
		self.fix=0

		#MSG
		self.fl_nw_msg=False
		self.msg=None

		#Drone position + yaw
		self.lat=None
		self.lon=None
		self.alt=None	#metres
		self.yaw=None	#degrees
		self.distWP=0	#metres

		#WP info
		self.WPlist= None
		self.WPDetec=None

		#State of video processing
		self.alg_en=False

		#Correctors PI
		self.corr_x=Corr_PI(Kp=0.01, Ki=0.001, Sats=(-2, 2) )
		self.corr_y=Corr_PI(Kp=0.01, Ki=0.001, Sats=(-2, 2) )


	#STATES methodes
	def st_init(self):
		#Transition INIT-->TAKEOFF
		if self.fix >= GPS_FX.FX_3D and self.alt<=0.1 and self.alt>=-0.1:
			#ACTION
			#Save WP
			self.WPlist=[ (self.lat, self.lon) ]
			#Takeoff
			print "Ready to takeoff"
			self.act_takeoff()
			#CHANGE ST
			self.state=FSM_ST.TAKEOFF



	def st_takeoff(self):
		#PRINT INFO
		print "Current altitude: ", self.alt, "m"
		
		#Transition TAKEOFF-->ERROR
		if self.st_exit:
			#ACTION
			print "Lost connection  --> error state"
			#CHANGE ST
			self.state=FSM_ST.ERROR

		#Transition TAKEOFF-->NAV
		elif self.alt>=self.tg_alt*0.95:
			#ACTION
			print "Going to: ", self.pt_gps_start
			self.act_nav_goto(self.pt_gps_start)

			#CHANGE ST
			self.state=FSM_ST.NAV


	def st_nav(self):

		#Transition NAV-->NAV
		if self.distWP>7 and not self.alg_en:
			#ACTION
			#Activate arrow algo
			Q_TX.put_bis(ALGOS.ARROW)
			self.alg_en=True
			#Print distance
			print "Distance from WP: ", self.distWP
			#CHANGE ST
			self.state=FSM_ST.NAV

		elif self.fl_nw_msg:

		#Transition NAV-->ARROW_TRK
			if self.msg.algo==ALGOS.ARROW and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				self.fl_nw_msg=False
				#Save detection point
				self.WPDetec=(self.lat, self.lon)
				#Init tracking
				self.act_tracking(reset=True)



	def st_arrow_trk(self):
		pass

	def st_arrow_algn(self):
		pass

	def st_squr_trk(self):
		pass

	def st_land(self):
		pass

	def st_error(self):
		pass


	#ACTION methodes
	def act_takeoff(self):
		print "Arming..."
		self.vehicle.mode    = VehicleMode("GUIDED")
		#Securite non inibe
		#vehicle.parameters["ARMING_CHECK"] = 0
		self.vehicle.armed   = True
		self.vehicle.flush()

		time.sleep(0.1)
		print "Waiting for arming cycle completes..."
		while not self.vehicle.armed and not self.api.exit:
			time.sleep(0.5)

		print "Taking off!"
		self.vehicle.commands.takeoff(self.tg_alt) # Take off to target altitude
		self.vehicle.flush()


	def act_nav_goto(self, gps_pt):
		(gt_lat, gt_lon) = gps_pt
		loc=Location(gt_lat, gt_lon, self.tg_alt, is_relative=True)
		self.vehicle.commands.goto(loc)
		self.vehicle.flush()


	def act_tracking(self, reset=False):
		#Reset corrector if needed
		if reset:
			self.corr_x.reset()
			self.corr_y.reset()

		#Compute speeds commands in UAV base
		(err_x, err_y)= self.msg.distances
		Vxd=self.corr_x.run(err_x)
		Vyd=self.corr_y.run(err_y)

		#Change of base: UAV --> earth
		(Vxa, Vya)=chg_base_abs(Vxd, Vyd, self.yaw)

		#Send commands to UAV
		self.send_nav_velocity(Vya, Vxa , 0)


	def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
		# create the SET_POSITION_TARGET_LOCAL_NED command
		# Check https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED
		# for info on the type_mask (0=enable, 1=ignore).
		# Accelerations and yaw are ignored in GCS_Mavlink.pde at the
		# time of writing.
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111000111, # type_mask (only speeds enabled)
			0, 0, 0, # x, y, z positions (not used)
			velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink.pde)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink.pde) 
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

	#ROUTINES methodes
	def rt_fsm_mng(self):

		#States functions selection
		if self.state == FSM_ST.INIT:
			self.st_init()
		elif self.state == FSM_ST.TAKEOFF:
			self.st_takeoff()
		elif self.state == FSM_ST.NAV:
			self.st_nav()
		elif self.state == FSM_ST.ARROW_TRK:
			self.st_arrow_trk()
		elif self.state == FSM_ST.ARROW_ALGN:
			self.st_takeoff()
		elif self.state == FSM_ST.SQUR_TRK:
			self.st_squr_trk()
		elif self.state == FSM_ST.LAND:
			self.st_land()

		#REINIT flags
		fl_nw_msg=False


	def rt_update_var(self):

		#Update api state
		self.st_exit= self.api.exit

		#Update GPS fix
		self.fix=self.vehicle.gps_0.fix_type

		#Update position
		self.lat=self.vehicle.location.lat
		self.lon=self.vehicle.location.lon
		self.alt=self.vehicle.location.alt

		#Get yaw
		self.yaw=self.vehicle.attitude.yaw

		#get distance from the last WP
		if self.WPlist!=None:
			lstWP_loc= self.WPlist[len(self.WPlist)-1]
			self.distWP=get_dst_2WP(lstWP_loc, (self.lat, self.lon))

		#Update msg from queues	
		(self.fl_nw_msg, self.msg)= Q_RX.get_bis()

