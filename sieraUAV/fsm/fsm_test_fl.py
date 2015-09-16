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
from misc.misc import *
from misc.geo_misc import chg_base_abs
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


	#STATES methodes
	def st_init(self):
		#Transition INIT-->TAKEOFF
		if self.fix >= GPS_FX.FX_3D and self.alt<=0.1 and self.alt>=-0.1:
			#ACTION
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
			print "Activate arrow algo"
			Q_TX.put_bis(ALGOS.ARROW)
			#CHANGE ST
			self.state=FSM_ST.NAV


	def st_nav(self):
		pass

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
		self.yaw=self.vehicle.attitude.yaw * 180/pi

		#Update msg from queues	
		(self.fl_nw_msg, self.msg)= Q_RX.get_bis()

