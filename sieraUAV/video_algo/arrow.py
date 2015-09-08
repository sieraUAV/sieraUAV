#!/usr/bin/python

from misc.kalman_tool import kalman2D
from misc.geo_misc   import *
from video_algo.algo_manage import *

"""
Return information about arrow detection algorytme
"""
class arrow_info:
	def __init__(self, status=STATUS_ALG.KO, dst=None, angle=None ):
		self.algo=ALGOS.ARROW
		self.status=status
		self.distances=dst
		self.yaw=angle

"""
Arrow detection algorytme
"""
class arrow:
	def __init__(self, align_pres=100):
		#init attributes

		#VAR POUR FILTRAGE
		self.countDeadDetect=0
		self.arrowK=kalman2D()

		#Presision pour le statut ALIGN
		self.align_pres=align_pres

	def processing(self, algo_man):

		#Update with algo management class
		img=algo_man.img
		algo_man.hud_size=self.align_pres
		rows=algo_man.rows
		cols=algo_man.cols

		#blur
		img = cv2.GaussianBlur(img,(5,5),0)
		 
		#HSV
		imageHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#NORMALISATEUR DE LUMINOSITE
		# separer les valeurs
		h,s,v = cv2.split(imageHSV)
		# Calcul de moyenne sur luminosite +offset
		meanValv=cv2.mean(v)
		diffv=125-meanValv[0]
		v=cv2.convertScaleAbs(v, beta=diffv)
		# refusioner l'image
		imageHSV = cv2.merge((h,s,v))

		#re-convertir en rgb
		imageHSV_hist = cv2.cvtColor(imageHSV, cv2.COLOR_HSV2BGR)
	 
		#thresholding
		min_red = np.array((0. ,125. ,70. ))
		max_red = np.array((10. ,255. ,255. ))

		 
		min_red2 = np.array((170. ,125. ,70. ))
		max_red2 = np.array((180. ,255. ,255. ))
	 
		imgThresh = cv2.inRange(imageHSV, min_red, max_red)
		imgThresh2= cv2.inRange(imageHSV, min_red2, max_red2)
		imgThreshT=cv2.bitwise_or(imgThresh,imgThresh2)
	 	
	 	#Update algo management class
	 	algo_man.imgTresh=imgThreshT

		#close function (effacer les trous)
		kernel = np.ones((7,7),np.uint8)
		closing = cv2.morphologyEx(imgThreshT, cv2.MORPH_CLOSE, kernel)
		 
		#contour
		contours,hier = cv2.findContours(closing,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	 
		#Filtrage de contour
		#init list element filtre
		cnt_filt=[]
		for cnt in contours:
			CurrAera=cv2.contourArea(cnt)
			if CurrAera>1500 :  # remove small areas like noise etc
				#hull = cv2.convexHull(cnt)    # find the convex hull of contour
				hull = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
				approx = cv2.convexHull(cnt)
				#if len(hull)==12:
				if not cv2.isContourConvex(hull) and len(hull)<=12:
					m = cv2.moments(hull)
					n = cv2.moments(approx)
					if m['m00'] !=0:
						barycentre=(int(m['m10']/m['m00']),int(m['m01']/m['m00']))
						cv2.drawContours(img,[hull],0,(0,0,255),2)
						barycentre_2=(int(n['m10']/n['m00']),int(n['m01']/n['m00']))

						cv2.circle(img,barycentre_2,4,(255,0,255),-1)
						cv2.drawContours(img,[approx],0,(0,255,255),2)
						cv2.line(img,barycentre,barycentre_2,[255,255,255],1)
						#register info 
						cnt_filt.append((CurrAera, barycentre, barycentre_2, hull))


		#trie des contour filtre par aire les plus grandre
		cnt_filt.sort(reverse=True)

		#DETECTION
		if len(cnt_filt)>0:

			#Put counter bad detection to 0
			self.countDeadDetect=0

			(a,b)=cnt_filt[0][2]
			(x,y)=(0,0)
			#Init kalman
			if not self.arrowK.alive:
				self.arrowK.init(a, b, m_noise_cov=1e-4)
			#Correct kalman
			else:
				self.arrowK.predict()
				(x,y)=self.arrowK.correct(a,b)

			ptx=(a,rows/2)
			pty=(cols/2,b)

			#Paint
			cv2.circle(img,(x,y),4,(0,255,0),-1)
			cv2.circle(img,cnt_filt[0][2],4,(255,0,0),-1)
			cv2.line(img,cnt_filt[0][2],ptx,[0,0,255],1)
			cv2.line(img,cnt_filt[0][2],pty,[0,0,255],1)
			cv2.drawContours(img,[cnt_filt[0][3]],0,(0,255,0),2)
			
			#Update img of algo management
			algo_man.img=img
			distances=interdistance(cnt_filt[0][1],cols,rows)

			#Test if we are align
			(distX,distY)=distances
			if abs(distX)<self.align_pres and abs(distY)<self.align_pres:
				#ALIGN
				yaw=angle(cnt_filt[0][1], cnt_filt[0][2])
				#Return info
				return arrow_info(status=STATUS_ALG.TRACKING, dst=distances, angle=yaw)
			
			else:
				#TRACKING
				#Return info
				return arrow_info(status=STATUS_ALG.TRACKING, dst=distances )

		#PREDICTION
		elif self.countDeadDetect<10 and self.arrowK.alive:

			#Increment counter
			self.countDeadDetect+=1


			(x,y)=self.arrowK.predict()

			#print prediction point
			cv2.circle(img,(x,y),4,(0,255,0),-1)

			distances=interdistance((x,y),cols,rows)
			
			#Return info
			algo_man.img=img
			return arrow_info(status=STATUS_ALG.TRACKING, dst=distances )

		#KALMAN DEAD
		else:
			self.arrowK.alive=False

			#Return info
			algo_man.img=img
			return arrow_info(status=STATUS_ALG.KO)
