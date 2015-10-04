#!/usr/bin/python
"""
@package This package to manage flight management FSM
"""
from math import pi, cos, sin
import numpy as np
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
from pymavlink import mavextra
from misc.th_misc import Q_TX, Q_RX
from video_algo.algo_manage import ALGOS, STATUS_ALG
from video_algo.cross import cross_info
from video_algo.blk_sqr import blk_sqr_info
from misc.misc import *
from misc.geo_misc import chg_base_abs, get_dst_2WP,angles_mean
import time 

GPS_FX=enum('KO', 'NO', 'FX_2D', 'FX_3D', 'DGPS', 'RTK')

FSM_ST=enum('INIT','TAKEOFF','NAV','CROSS_TRK', 'CROSS_DROP', 'SQUR_SRCH', 'SQUR_TRK', 'LAND', 'RECOVER', 'ERROR')

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
		self.WPTarget=None

		#State of video processing
		self.alg_en=False
		self.alignCounter=0

		#ALIGN INFO
		self.angles=list()

		#TIMER
		self.lst_time=None

		#TRACKING
		#timer act tracking
		self.lst_time_track=None
		#Freq for tracking wth goto (Hz)
		self.freqTrk=1

	#STATES methodes
	def st_init(self):
		#Transition INIT-->TAKEOFF
		print "altitute:",self.alt
		if self.fix >= GPS_FX.FX_3D and self.alt<=1 and self.alt>=-1:
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
			#Goto to the first WP
			print "Going to: ", self.pt_gps_start
			self.act_nav_goto(self.pt_gps_start)
			#Save target Wp
			self.WPTarget=self.pt_gps_start
			#CHANGE ST
			self.state=FSM_ST.NAV


	def st_nav(self):

		#Transition NAV-->NAV
		if self.distWP>10 and not self.alg_en:
			#ACTION
			#Activate arrow algo
			Q_TX.put_bis(ALGOS.CROSS)
			self.alg_en=True
			#Print distance
			print "Distance from WP: ", self.distWP
			#CHANGE ST
			self.state=FSM_ST.NAV

		elif self.fl_nw_msg:
			self.fl_nw_msg=False

			#Transition NAV-->ARROW_TRK
			if self.msg.algo==ALGOS.CROSS and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Save detection point
				self.WPDetec=(self.lat, self.lon)
				#Init tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.CROSS_TRK

	def st_cross_trk(self):
		#Transition ARROW_TRK-->ARROW_ALIGN
		if self.alignCounter>5:
			#ACTION
			self.alignCounter=0
			#CHANGE ST
			self.state=FSM_ST.CROSS_DROP

		elif self.fl_nw_msg:
			self.fl_nw_msg=False

			#Transition ARROW_TRK-->ARROW_TRK
			if self.msg.algo==ALGOS.CROSS and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Count align msg
				if self.msg.status==STATUS_ALG.DROPPING:
					self.alignCounter+=1
				else:
					self.alignCounter=0
				#tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.CROSS_TRK

			#Transition ARROW_TRK-->RECOVER
			elif self.msg.algo==ALGOS.CROSS and self.msg.status==STATUS_ALG.KO:
				#ACTION
				#Goto to last detection WP
				self.act_nav_goto(self.WPDetec)
				#Start timer
				self.lst_time=time.time()
				#CHANGE ST
				self.state=FSM_ST.RECOVER


	def st_cross_drop(self):
		#Transition CROSS_DROP-->SQUR_SRCH
		#ACTION
		print "Drop 1"
		self.send_servo(10,1)
		time.sleep(2)
		print "Drop 2"
		self.send_servo(10,2)
		time.sleep(2)
		print "Servo in close position"
		self.send_servo(10,0)
		#Re-take navigation
		self.act_nav_goto(self.pt_gps_start)
		#Activate black square algo
		Q_TX.put_bis(ALGOS.BLK_SQUR)

		#CHANGE STATE
		self.state=FSM_ST.SQUR_SRCH
			


	def st_squr_srch(self):
		if self.fl_nw_msg:
			self.fl_nw_msg=False

			#Transition SQR_srch-->squr_trk
			if self.msg.algo==ALGOS.BLK_SQUR and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Save detection point
				self.WPDetec=(self.lat, self.lon)
				#Init tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.SQUR_TRK

	def st_squr_trk(self):
		
		#Transition SQUR_TRK-->LAND
		if self.alignCounter>5:
			#ACTION
			self.alignCounter=0
			#change mode to land
			self.vehicle.mode    = VehicleMode("LAND")
			#CHANGE ST
			self.state=FSM_ST.LAND
			print('go to land')

		elif self.fl_nw_msg:
			self.fl_nw_msg=False

			#Transition SQUR_TRK-->SQUR_TRK
			if self.msg.algo==ALGOS.BLK_SQUR and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Count align msg
				if self.msg.status==STATUS_ALG.LAND:
					print('yolo')
					self.alignCounter+=1
				else:
					self.alignCounter=0
				#tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.SQUR_TRK

			#Transition SQUR_TRK-->RECOVER
			elif self.msg.algo==ALGOS.BLK_SQUR and self.msg.status==STATUS_ALG.KO:
				#ACTION
				#Goto to last detection WP
				self.act_nav_goto(self.WPDetec)
				#Start timer
				self.lst_time=time.time()
				#CHANGE ST
				self.state=FSM_ST.RECOVER

	def st_land(self):
		print "Current altitude: ", self.alt, "m"
		#print('landing')
		#self.act_land()
		#self.state=FSM_ST.LAND
		pass


	def st_recover(self):
		dstDtec=get_dst_2WP(self.WPDetec, (self.lat, self.lon))
		boolTimeout= (time.time()-self.lst_time)>15

		if self.fl_nw_msg:
			self.fl_nw_msg=False
			#Transition RECOVER-->ARROW_TRK
			if self.msg.algo==ALGOS.CROSS and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Reset timer
				self.lst_time=None
				#Save detection point
				self.WPDetec=(self.lat, self.lon)
				#Init tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.CROSS_TRK

			#Transition RECOVER-->SQUR_TRK
			elif self.msg.algo==ALGOS.BLK_SQUR and self.msg.status!=STATUS_ALG.KO:
				#ACTION
				#Reset timer
				self.lst_time=None
				#Save detection point
				self.WPDetec=(self.lat, self.lon)
				#Init tracking
				self.act_tracking()
				#CHANGE ST
				self.state=FSM_ST.SQUR_TRK

			#Transition RECOVER-->NAV
			elif self.msg.status==STATUS_ALG.KO and dstDtec<0.5 and boolTimeout:
				#ACTION
				#Reset timer
				self.lst_time=None
				#Goto to the target WP
				self.act_nav_goto(self.WPTarget)
				#CHANGE ST
				self.state=FSM_ST.NAV


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
		while not self.vehicle.armed :
			time.sleep(5)
			self.vehicle.armed   = True
			self.vehicle.flush()

		print "Taking off!"
		self.vehicle.commands.takeoff(self.tg_alt) # Take off to target altitude
		self.vehicle.flush()


	def act_nav_goto(self, gps_pt):
		(gt_lat, gt_lon) = gps_pt
		loc=Location(gt_lat, gt_lon, self.tg_alt, is_relative=True)
		self.vehicle.commands.goto(loc)
		self.vehicle.flush()
		

	def act_tracking(self):

		if self.lst_time_track==None:
			timer=1/self.freqTrk+0.1
		else:
			timer=time.time()-self.lst_time_track

		if timer>(1/self.freqTrk) :
			#Save current time for timer
			self.lst_time_track=time.time()

			#Camera def
			angl_viewH= 75.7*pi/180.
			angl_viewV= 66.*pi/180.

			#Compute position corection (UAV base)
			(err_x_rt, err_y_rt)= self.msg.distances
			dsterrXd= (angl_viewH*err_x_rt)*self.alt
			dsterrYd= (angl_viewV*err_y_rt)*self.alt

			#Change of base: UAV --> earth
			(dsterrXa, dsterrYa)=chg_base_abs(dsterrXd, dsterrYd, self.yaw)
			#Compute new Waypoint for track
			poscmd=mavextra.gps_offset(self.lat, self.lon, dsterrXa, dsterrYa )

			#Send commands to UAV
			self.act_nav_goto(poscmd)


	def act_get_angle(self):
		if self.msg.status==STATUS_ALG.ALIGN:
			rel_angle=self.msg.angle
			#Compute angle
			abs_angle= (rel_angle+self.yaw*180./pi)%360
			#Save angle
			self.angles.append(abs_angle)
			#Check if angle is correct
			(ret,meanangl)=angles_mean(self.angles)
			if ret:
				#Bad angle --> reset list
				self.angles=list()


	def act_goto_wt_angle(self):

		#Get angles average
		(ret,bearing)=angles_mean(self.angles)
		print "List angles", self.angles
		print "Current yaw", (self.yaw*180./pi)
		if ret:
			print "ERROR BAD ANGLES"
		else:
			#Save current WP
			currWP= (self.lat, self.lon)
			#Get WP target
			self.WPTarget=mavextra.gps_newpos(self.lat, self.lon, bearing, 30)
			#Goto to target 
			self.act_nav_goto(self.WPTarget)
			time.sleep(3)

			currpos=(self.vehicle.location.lat, self.vehicle.location.lon)
			while get_dst_2WP(currWP, currpos) < 3:
				print "Retry to send cmd"
				self.act_nav_goto(self.WPTarget)
				time.sleep(3)
				currpos=(self.vehicle.location.lat, self.vehicle.location.lon)

			dstDtar=get_dst_2WP(self.WPTarget, (self.lat, self.lon))
			print "Distance to target %d m" % int(dstDtar)
			#Reset angle list
			self.angles=list()






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

	def send_servo( self, servo, stategates):
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

		msg = self.vehicle.message_factory.command_long_encode(
			0, 0,                                   # target system, target component
			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,   #command
			0,  #confirmation
			servo,  #param 1
			pwm,    #param 2
			0, 0, 0, 0, 0)   

		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

	#ROUTINES methodes
	def rt_fsm_mng(self):
		last_st=self.state

		#States functions selection
		if self.state == FSM_ST.INIT:
			self.st_init()
		elif self.state == FSM_ST.TAKEOFF:
			self.st_takeoff()
		elif self.state == FSM_ST.NAV:
			self.st_nav()
		elif self.state == FSM_ST.CROSS_TRK:
			self.st_cross_trk()
		elif self.state == FSM_ST.CROSS_DROP:
			self.st_cross_drop()
		elif self.state == FSM_ST.SQUR_SRCH:
			self.st_squr_srch()
		elif self.state == FSM_ST.SQUR_TRK:
			self.st_squr_trk()
		elif self.state == FSM_ST.LAND:
			self.st_land()
		elif self.state == FSM_ST.RECOVER:
			self.st_recover()

		#Print changes of states
		if self.state!=last_st:
			print "Change of state. Going to %d state" % self.state

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

