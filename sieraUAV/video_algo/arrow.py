#!/usr/bin/python

from misc.kalman_tool import kalman2D
from misc.geo_misc   import *
from video_algo.algo_manage import *
from misc.vid_misc import *

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
		self.arrowK=kalman2D(alive=False)
		self.tresh=clever_thresh()

		#Presision pour le statut ALIGN
		self.align_pres=align_pres


	def angle_PCA_Ctr(self, contour):
		#PCA
		bufdata=np.array([contour[0][0]])
		for n in range(1,len(contour)):
			bufdata= np.append(bufdata, [contour[n][0]], axis=0)

		#Compute PCA
		(data, eigenvalues, eigenvectors)=PCA(bufdata)

		#Determine front
		cnt_pos_x=0
		invert=1.
		for pt in data:
			if pt[0]>0:
				cnt_pos_x+=1
			else:
				cnt_pos_x-=1

		if cnt_pos_x<0:
			invert=-1.

		#extract main direction vector
		v1=eigenvectors[0]*invert
		v1[1]*=-1

		#compute angle
		yaw=angle((0.,0.), (v1[0], v1[1]))

		return (yaw, v1, eigenvalues)


	def display_info(self, img, state="KO", dst=None, angle=None):

		string= "Tracking informations: %s" % state

		if dst!=None:
			string+= ", Position error: x=%d y=%d" % dst

		if angle!=None:
			string+= ", Arrow angle: %d" % int(angle)

		#Print info on img
		cv2.putText(img,string, (5,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10,255,10),1)


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
		# # separer les valeurs
		# h,s,v = cv2.split(imageHSV)
		# # Calcul de moyenne sur luminosite +offset
		# meanValv=cv2.mean(v)
		# diffv=125-meanValv[0]
		# v=cv2.convertScaleAbs(v, beta=diffv)
		# # refusioner l'image
		# imageHSV = cv2.merge((h,s,v))

		#re-convertir en rgb
		
		imgHSV2=np.array(imageHSV)
	 
		imgThreshT=self.tresh.exe(imageHSV)
	 	
	 	#Update algo management class
	 	algo_man.imgTresh=np.array(imgThreshT)

		#close function (effacer les trous)
		kernel = np.ones((4	,4),np.uint8)
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
				if not cv2.isContourConvex(hull) and len(hull)>=6 and len(hull)<=10:
					m = cv2.moments(hull)
					n = cv2.moments(approx)
					if m['m00'] !=0:
						barycentre=(int(m['m10']/m['m00']),int(m['m01']/m['m00']))
						cv2.drawContours(img,[hull],0,(0,0,255),2)
						barycentre_2=(int(n['m10']/n['m00']),int(n['m01']/n['m00']))

						cv2.circle(img,barycentre_2,4,(255,0,255),-1)
						cv2.drawContours(img,[approx],0,(0,255,255),2)
						#register info 
						cnt_filt.append((CurrAera, barycentre, barycentre_2, hull))


		#trie des contour filtre par aire les plus grandre
		cnt_filt.sort(reverse=True)

		#DETECTION
		if len(cnt_filt)>0:

			#Put counter bad detection to 0 or decremente
			if self.countDeadDetect>0:
				self.countDeadDetect=0
			elif self.countDeadDetect<=-6:
				self.countDeadDetect=-6
			else:
				self.countDeadDetect-=1

			(xd,yd)=cnt_filt[0][2]


			#Update clever threshold and kalman if we need (after 5 detection)
			if self.countDeadDetect<=-3:
				#KALMAN
				if not self.arrowK.alive:
					self.arrowK.__init__(xd, yd , True, m_noise_cov=1e-3	, dst_des=rows*0.20 )

			if self.arrowK.alive:
				#Correct kalman 
				(xk,yk,ret_corr)=self.arrowK.correct(xd,yd)
				cv2.circle(img,(xk,yk),4,(0,255,0),-1)

				#If correction sucess
				if self.arrowK.track :

					ptx=(xk,rows/2)
					pty=(cols/2,yk)

					#Paint distance between kalman prediction and center of the picture
					cv2.line(img,(xk,yk),ptx,[0,0,255],1)
					cv2.line(img,(xk,yk),pty,[0,0,255],1)

					if ret_corr:
						#Update clever thresh
						self.tresh.compute_HSV_pt(cnt_filt[0][1], imgHSV2, img, (cnt_filt[0][0]*0.03)**0.5)
						##Paint rest of info##
						#Detection barycenter
						cv2.circle(img,cnt_filt[0][2],4,(255,0,0),-1)
						#Detection contour
						cv2.drawContours(img,[cnt_filt[0][3]],0,(0,255,0),2)
						#Validation circle
						cv2.circle(img,(xk,yk),int(self.arrowK.dst_des),(0,0,255),1)




					#Update img of algo management
					algo_man.img=img

					distances=interdistance((xk,yk),cols,rows)

					#Test if we are align
					(distX,distY)=distances
					if abs(distX)<self.align_pres and abs(distY)<self.align_pres and ret_corr:
						#ALIGN
						#PCA angle calculation
						(yaw, main_vec,  eigenvalues)=self.angle_PCA_Ctr(cnt_filt[0][3])

						#draw direction
						pt_ctr=cnt_filt[0][2]
						center= np.array([float(pt_ctr[0]), float(pt_ctr[1])])
						p1=center+50.*main_vec
						p1= (int(p1[0]), int(p1[1]))
						cv2.line(img,pt_ctr,p1,[255,0,0],2)

						#Print info
						self.display_info(img, "ALIGN", distances, yaw)
						#Return info
						return arrow_info(status=STATUS_ALG.ALIGN, dst=distances, angle=yaw)
					
					else:
						#TRACKING
						#Print info
						self.display_info(img, "TRACKING", distances)
						#Return info
						return arrow_info(status=STATUS_ALG.TRACKING, dst=distances )


				else:
					#Update img of algo management
					algo_man.img=img

			else:
				#if not tracking --> return to basic tresh
				self.tresh.status=STAT_THR.DEF

		#PREDICTION
		elif self.countDeadDetect<10 and self.arrowK.track:

			#Increment counter
			self.countDeadDetect+=1

			if self.countDeadDetect<=0:
				self.tresh.load_old_HSV_pt()
			else:
				self.tresh.status=STAT_THR.DEF

			#Get prediction point
			(x,y)=self.arrowK.predict()

			#print prediction point
			cv2.circle(img,(x,y),4,(0,255,0),-1)

			distances=interdistance((x,y),cols,rows)


			#Return info
			algo_man.img=img
			#Print info
			self.display_info(img, "TRACKING", distances)
			return arrow_info(status=STATUS_ALG.TRACKING, dst=distances )

		#KALMAN DEAD
		else:
			self.arrowK.alive=False

			#Return info
			algo_man.img=img

			#Update clever threshold
			self.tresh.status=STAT_THR.DEF

		#Print info
		self.display_info(img)
		#Default return	
		return arrow_info(status=STATUS_ALG.KO)	
