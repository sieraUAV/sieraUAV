#!/usr/bin/python

import time

"""
@package This package implement basic tools (like enum)
"""

"""
Function to create a basic enum 
"""
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

'''
PI corrector object (clamping integretor)
'''
class Corr_PI:
	def __init__(self, Kp, Ki, Sats):
		#Corrector config
		self.Kp=Kp		#Propertional gain
		self.Ki=Ki		#Integrator gain
		self.Sats=Sats

		#Corrector internal var
		self.inte_value=0
		self.last_time=None
		self.st_clamping=True

	def run(self, err):
		#Var
		con_Kp=0
		con_Ki=0
		con_sum=0
		con_sat=0
		# get time
		corr_time=time.time()

		#Correction
		#P
		con_Kp=self.Kp*err
		#I
		if self.st_clamping:
			con_Ki=0
		else:
			self.inte_value+=self.Ki*err*(corr_time-self.st_clamping)
			con_Ki=self.inte_value

		#SUM
		con_sum=con_Kp+con_Ki

		#Saturation
		(low, hight)=self.Sats
		if con_sum>hight:
			con_sat=hight
		elif con_sum<low:
			con_sat=low
		else:
			con_sat=con_sum

		#Determination of clamping: sat and err(t)*u(t)>0

		if con_sum!=con_sat and (err*con_sat)>0:
			self.st_clamping=True
		else:
			self.st_clamping=False

		#Save time
		self.last_time=corr_time

		#Return command
		return con_sat

	def reset(self):
		#Reset corrector internal var
		self.inte_value=0
		self.last_time=None
		self.st_clamping=True


