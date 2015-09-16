#!/usr/bin/python
"""
@package This package drive algo
"""
import cv2
import time
import Image
import math
import numpy as np

from misc.misc import enum

#ENUM
ALGOS=enum('NONE','ARROW','CROSS','BLK_SQUR')
STATUS_ALG= enum('KO','TRACKING','ALIGN')

#CLASS

class algo_process:
	def __init__(self, camera=-1, def_allgo=ALGOS.ARROW, hud_size=100):
		##init attributes##
		#img properties
		self.cols=None
		self.rows=None
		self.channels=None
		#img buffer
		self.img=None
		self.imgTresh=None
		#Current algo
		self.sel_algo=def_allgo
		#Enregis video
		self.boolVideo=False
		self.out=None
		#Capture
		self.capture=None
		#Stop capture
		self.boolStop=False
		#HUD size
		self.hud_size=hud_size

		##init capture##
		self.init_cv(camera)

	def __del__(self):
		self.capture.release()
    	cv2.destroyAllWindows()

	def init_cv(self, camera):
		#INIT WINDOW
		cv2.namedWindow("Composed", cv2.CV_WINDOW_AUTOSIZE)
		cv2.startWindowThread()

		#CONFIG CAPTURE
		self.capture=cv2.VideoCapture(camera)
		
		#CONFIG CAPTURE VIDEO
		w=int(self.capture.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH ))
		h=int(self.capture.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT ))
		print 'hauteur %d et largeur %d' % (h ,w)
		self.out = cv2.VideoWriter('output.avi',cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 10,(2*w , h),1)

		#test capture
		self.capture.read()
		ret, img = self.capture.read()
		rows,cols,channels = img.shape
		while not ret:
			ret, img = self.capture.read()
    		rows,cols,channels = img.shape

		self.cols=cols
		self.rows=rows

	def read(self):
		ret, Retimg = self.capture.read()
		
		if ret:
			self.img=Retimg

		return ret

	def display(self):

		if self.img!=None and self.imgTresh!=None:
			#HUD
			self.drawHUD()
			#CREATE COMPOSED IMAGE
			compoImage = np.zeros((self.rows,2*self.cols,3), np.uint8)
			compoImage[0:self.rows, 0:self.cols ] = self.img
			imgThreshTRGB=cv2.cvtColor ( self.imgTresh, cv2.COLOR_GRAY2BGR )
			compoImage[0:self.rows, self.cols:2*self.cols ] = imgThreshTRGB
			#CAPTURE VIDEO
			if self.boolVideo:
				self.out.write(compoImage)
				cv2.circle(compoImage,(20,20),10,(0,0,255),-1)
			
			cv2.imshow("Composed", compoImage)

	def drawHUD(self):
		#repere
		pt1=(self.cols/2,0)
		pt2=(self.cols/2,self.rows)
		cv2.line(self.img,pt1,pt2,[255,255,255],1)
		pt3=(0,self.rows/2)
		pt4=(self.cols,self.rows/2)
		cv2.line(self.img,pt3,pt4,[255,255,255],1)

		#draw square
		sq1= (self.cols/2 + self.hud_size, self.rows/2 + self.hud_size)
		sq2= (self.cols/2 + self.hud_size, self.rows/2 - self.hud_size)
		sq3= (self.cols/2 - self.hud_size, self.rows/2 - self.hud_size)
		sq4= (self.cols/2 - self.hud_size, self.rows/2 + self.hud_size)

		cv2.line(self.img,sq1,sq2,[255,255,255],1)
		cv2.line(self.img,sq2,sq3,[255,255,255],1)
		cv2.line(self.img,sq3,sq4,[255,255,255],1)
		cv2.line(self.img,sq4,sq1,[255,255,255],1)

	def toucheAction(self, delai=10):
     
		flag=True   
		"""
		numero de touche
		's'--> sauvgarde une image
		'ESC' --> quitte le programe
		'g' --> demarre ou arrete la capture
		"""

		#conf 1
		#img_cpt=1048691
		#esp=1048603
		#video=1048679

		#conf 2
		img_cpt=115
		esp=27
		video=118
		pause=112

		PAUSE=False

		k = cv2.waitKey(delai)

		if k!=-1:
			print 'touche:',k

		if k==img_cpt:
			cv2.imwrite('capture.png',self.img)
		elif k == esp:         # wait for ESC key to exit
			self.boolStop=True
		elif k == video:
			self.boolVideo=not self.boolVideo
		elif k==pause:
			PAUSE=True
			while PAUSE:
				k = cv2.waitKey(delai)
				if k==pause:
					PAUSE=False
		else:
			flag=False
		return flag
