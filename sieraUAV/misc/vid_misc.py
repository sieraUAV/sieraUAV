#!/usr/bin/python

"""
@package This package contains the image processing tool (threshold etc...)
"""
import math
import cv2
import numpy as np
from misc import *

#ENUM
STAT_THR= enum('DEF','TRACK')
COLOR= enum('RED','YELLOW')

"""	
Adaptive threshold in function of a point
"""
class clever_thresh:
	def __init__( self, V_per=20, S_per=10, vec_val_per=8, color=COLOR.RED ):
		self.V_per=V_per
		self.S_per=S_per
		self.vec_val_per=vec_val_per
		self.status=STAT_THR.DEF
		self.color=color

		#REF POINT
		self.HSV_pt_val=None
		self.HSV_pt_val_old=None

		#COLOR VECTOR
		#Valeur par default: (valeur_def_couleur, min_def_saturation, min_def_valeur)
		self.default_red= (158., 20., 125.)
		self.default_yellow= (28., 50., 10.)
		self.maxS=230.
		self.maxV=255.
		self.minS=5.
		self.minV=5.

	def exe(self, imgHSV):
		#VAR
		imgTresh=None

		#TRACKING
		if self.status== STAT_THR.TRACK:
			if self.color==COLOR.RED:
				imgTresh=self.color_th_pt(self.HSV_pt_val, self.default_red, imgHSV)
			elif self.color==COLOR.YELLOW:
				imgTresh=self.color_th_pt(self.HSV_pt_val, self.default_yellow, imgHSV)
			
			if imgTresh==None:
				self.status=STAT_THR.DEF
			else:
				return imgTresh

		#DEFAULT VALUES
		if self.status==STAT_THR.DEF:
			#Re-init HSV_pt
			self.HSV_pt_val=None
			self.HSV_pt_val_old=None
			#COLOR FILTER
			if self.color==COLOR.RED:
				imgTresh=self.color_th_def(self.default_red, imgHSV)
			elif self.color==COLOR.YELLOW:
				imgTresh=self.color_th_def(self.default_yellow, imgHSV)
		
		return imgTresh

	def color_th_def(self, def_color, img_HSV):
		#VAR
		imgHSV=np.array(img_HSV)
		imgTresh=None
		imgTresh1=None
		imgTresh2=None
		minH=None
		maxH=None
		ve_max1=None
		ve_max2=None
		ve_min1=None
		ve_min2=None

		(h,s,v)=def_color
		minH= float(h-180*self.vec_val_per/100)
		maxH= float(h+180*self.vec_val_per/100)

		#check hue cases
		if maxH>179:
			maxH-=180
			ve_min1=np.array((0. ,s ,v ))
			ve_max1=np.array((maxH ,self.maxS ,self.maxV ))
			ve_min2=np.array((minH ,s ,v ))
			ve_max2=np.array((179. ,self.maxS ,self.maxV ))
			imgTresh1 = cv2.inRange(imgHSV, ve_min1, ve_max1)
			imgTresh2= cv2.inRange(imgHSV, ve_min2, ve_max2)
			imgTresh=cv2.bitwise_or(imgTresh1,imgTresh2)
		elif minH<0:
			minH+=180
			ve_min1=np.array((0. ,s ,v ))
			ve_max1=np.array((maxH ,self.maxS ,self.maxV ))
			ve_min2=np.array((minH ,s ,v ))
			ve_max2=np.array((179. ,self.maxS ,self.maxV ))
			imgTresh1 = cv2.inRange(imgHSV, ve_min1, ve_max1)
			imgTresh2= cv2.inRange(imgHSV, ve_min2, ve_max2)
			imgTresh=cv2.bitwise_or(imgTresh1,imgTresh2)
		else:
			ve_min1=np.array((minH ,s ,v ))
			ve_max1=np.array((maxH ,self.maxS ,self.maxV ))
			imgTresh= cv2.inRange(imgHSV, ve_min1, ve_max1)

		return imgTresh

	def color_th_pt(self, pt_color, def_color, img_HSV):
		#VAR
		imgHSV=np.array(img_HSV)
		imgTresh=None
		imgTresh1=None
		imgTresh2=None
		minH=None
		maxH=None
		ve_max1=None
		ve_max2=None
		ve_min1=None
		ve_min2=None

		(h,s,v)=pt_color
		(hd,sd,vd)=def_color

		##Saturation Value min max##
		minS=float(s-255.*self.S_per/100)
		maxS=float(s+255.*self.S_per/100)
		minV=float(v-255.*self.V_per/100)
		maxV=float(v+255.*self.V_per/100)


		#Check min max for saturation and value
		if maxS>self.maxS:
			maxS=self.maxS
		if maxV>self.maxV:
			maxV=self.maxV
		if minS<self.minS:
			minS=self.minS
		if minV<self.minV:
			minV=self.minV

		##Hue min max##
		minH= float(h-180*self.vec_val_per/100)
		maxH= float(h+180*self.vec_val_per/100)

		minHd= float(hd-180*self.vec_val_per/100)
		maxHd= float(hd+180*self.vec_val_per/100)

		#Check if pt vector hue and default vector hue are compatible
		if maxH<minHd or maxHd<minH:
			return None

		#Check hue cases
		if maxH>180:
			maxH-=180
			ve_min1=np.array((0. ,minS ,minV ))
			ve_max1=np.array((maxH ,maxS ,maxV ))
			ve_min2=np.array((minH ,minS ,minV ))
			ve_max2=np.array((179. ,maxS ,maxV ))
			imgTresh1 = cv2.inRange(imgHSV, ve_min1, ve_max1)
			imgTresh2= cv2.inRange(imgHSV, ve_min2, ve_max2)
			imgTresh=cv2.bitwise_or(imgTresh1,imgTresh2)
		elif minH<0:
			minH+=180
			ve_min1=np.array((0. ,minS ,minV ))
			ve_max1=np.array((maxH ,maxS ,maxV ))
			ve_min2=np.array((minH ,minS ,minV ))
			ve_max2=np.array((179. ,maxS ,maxV ))
			imgTresh1 = cv2.inRange(imgHSV, ve_min1, ve_max1)
			imgTresh2= cv2.inRange(imgHSV, ve_min2, ve_max2)
			imgTresh=cv2.bitwise_or(imgTresh1,imgTresh2)
		else:
			ve_min1=np.array((minH ,minS ,minV ))
			ve_max1=np.array((maxH ,maxS ,maxV ))
			imgTresh= cv2.inRange(imgHSV, ve_min1, ve_max1)


		return imgTresh


	def compute_HSV_pt(self, pt_src, img_HSV, img=None, width=20 ):
		imgHSV=np.array(img_HSV)

		(cols_off, rows_off)=pt_src
		rows,cols,channels = imgHSV.shape

		minR=rows_off-int(width/2)
		maxR=rows_off+int(width/2)

		minC=cols_off-int(width/2)
		maxC=cols_off+int(width/2)

		#Check min max
		if minR<0:
			minR=0
		if maxR>rows:
			maxR=rows
		if minC<0:
			minC=0
		if maxC>cols:
			maxC=cols

		if img!=None:
			cv2.line(img,(minC, minR),(maxC, minR),[255,255,255],1)
			cv2.line(img,(maxC, minR),(maxC,maxR),[255,255,255],1)
			cv2.line(img,(maxC,maxR),(minC, maxR),[255,255,255],1)
			cv2.line(img,(minC, maxR),(minC, minR),[255,255,255],1)

		img_part=np.zeros((width,width,3), np.uint8)
		img_part= imgHSV[minR:maxR, minC:maxC]



		#Get avrages (hue, saturation, value)
		h,s,v = cv2.split(img_part)
		h=cv2.mean(h)[0]
		s=cv2.mean(s)[0]
		v=cv2.mean(v)[0]

		#Change status
		self.status=STAT_THR.TRACK

		#Update color vector to track object
		if self.HSV_pt_val!=None:
			self.HSV_pt_val_old=list(self.HSV_pt_val)
		self.HSV_pt_val=(h,s,v)

	def load_old_HSV_pt(self):
		#try to load last good threshold config
		if self.HSV_pt_val_old!=None:
			self.HSV_pt_val=list(self.HSV_pt_val_old)
		else:
			self.status=STAT_THR.DEF