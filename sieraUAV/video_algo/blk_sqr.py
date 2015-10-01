#!/usr/bin/python

from misc.kalman_tool import kalman2D
from misc.geo_misc   import *
from video_algo.algo_manage import *
from misc.vid_misc import *

"""
Return information about arrow detection algorytme
"""
class blk_sqr_info:
	def __init__(self, status=STATUS_ALG.KO, dst=None, angle=None ):
		self.algo=ALGOS.BLK_SQUR
		self.status=status
		self.distances=dst

"""
Arrow detection algorytme
"""
class blk_sqr:
	def __init__(self, align_pres=100):
		#init attributes

		#VAR POUR FILTRAGE
		self.countDeadDetect=0
		self.blk_sqrK=kalman2D(alive=False)

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

		string= "Tracking informations: SQUARE %s" % state

		if dst!=None:
			string+= ", Position error: x=%d y=%d" % dst


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
		 
		#GRAY img
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		MIN_LIGHT=0
		MAX_LIGHT=45
		#Threshold gray
		gray_bin=cv2.inRange(gray,MIN_LIGHT,MAX_LIGHT)
	 	
	 	#Update algo management class
	 	algo_man.imgTresh=np.array(gray_bin)

		#close function (effacer les trous)
		kernel = np.ones((5	,5),np.uint8)
		closing = cv2.morphologyEx(gray_bin, cv2.MORPH_CLOSE, kernel)
		 
		#contour
		contours,hier = cv2.findContours(closing,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	 
		#Filtrage de contour
		#init list element filtre
		cnt_filt=[]
		for cnt in contours:
			CurrAera=cv2.contourArea(cnt)
			if CurrAera>1500 :  # remove small areas like noise etc
				#hull = cv2.convexHull(cnt)    # find the convex hull of contour
				hull = cv2.approxPolyDP(cnt,0.10*cv2.arcLength(cnt,True),True)
				cv2.drawContours(img,[hull],0,(0,0,255),2)
				#if len(hull)==12:
				if cv2.isContourConvex(hull) and len(hull)==4:
					m = cv2.moments(hull)
					if m['m00'] !=0:
						barycentre=(int(m['m10']/m['m00']),int(m['m01']/m['m00']))
						cv2.drawContours(img,[hull],0,(0,0,255),2)
						#register info 
						cnt_filt.append((CurrAera, barycentre, hull))


		#trie des contour filtre par aire les plus grandre
		cnt_filt.sort(reverse=True)

				#Update with algo management class
		algo_man.img=img

		#DETECTION
		if len(cnt_filt)>0:

			#Put counter bad detection to 0 or decremente
			if self.countDeadDetect>0:
				self.countDeadDetect=0
			elif self.countDeadDetect<=-6:
				self.countDeadDetect=-6
			else:
				self.countDeadDetect-=1

			(xd,yd)=cnt_filt[0][1]


			#Update kalman if we need (after 5 detection)
			if self.countDeadDetect<=-3:
				#KALMAN
				if not self.blk_sqrK.alive:
					self.blk_sqrK.__init__(xd, yd , True, m_noise_cov=1e-3	, dst_des=rows*0.20 )

			if self.blk_sqrK.alive:
				#Correct kalman 
				(xk,yk,ret_corr)=self.blk_sqrK.correct(xd,yd)
				cv2.circle(img,(xk,yk),4,(0,255,0),-1)

				#If correction sucess
				if self.blk_sqrK.track :

					ptx=(xk,rows/2)
					pty=(cols/2,yk)

					#Paint distance between kalman prediction and center of the picture
					cv2.line(img,(xk,yk),ptx,[0,0,255],1)
					cv2.line(img,(xk,yk),pty,[0,0,255],1)

					if ret_corr:
						#Detection barycentre
						cv2.circle(img,cnt_filt[0][1],4,(255,0,0),-1)
						#Detection contour
						cv2.drawContours(img,[cnt_filt[0][2]],0,(0,255,0),2)
						#Validation circle
						cv2.circle(img,(xk,yk),int(self.blk_sqrK.dst_des),(0,0,255),1)




					#Update img of algo management
					algo_man.img=img

					distances=interdistance((xk,yk),cols,rows)
					dst_ratio=(float(distances[0])/cols, float(distances[1])/rows)

					#Test if we are align
					(distX,distY)=distances
					if abs(distX)<self.align_pres and abs(distY)<self.align_pres and ret_corr:
						#ALIGN

						#Print info
						self.display_info(img, "ALIGN", distances)
						#Return info
						return blk_sqr_info(status=STATUS_ALG.ALIGN, dst=dst_ratio)
					
					else:
						#TRACKING
						#Print info
						self.display_info(img, "TRACKING", distances)
						#Return info
						return blk_sqr_info(status=STATUS_ALG.TRACKING, dst=dst_ratio )


				else:
					#Update img of algo management
					algo_man.img=img

		#PREDICTION
		elif self.countDeadDetect<10 and self.blk_sqrK.track:

			#Increment counter
			self.countDeadDetect+=1

			#Get prediction point
			(x,y)=self.blk_sqrK.predict()

			#print prediction point
			cv2.circle(img,(x,y),4,(0,255,0),-1)

			distances=interdistance((x,y),cols,rows)
			dst_ratio=(float(distances[0])/cols, float(distances[1])/rows)

			#Return info
			algo_man.img=img
			#Print info
			self.display_info(img, "TRACKING", distances)
			return blk_sqr_info(status=STATUS_ALG.TRACKING, dst=dst_ratio )

		#KALMAN DEAD
		else:
			self.blk_sqrK.alive=False

			#Return info
			algo_man.img=img


		#Print info
		self.display_info(img)
		#Default return	
		return blk_sqr_info(status=STATUS_ALG.KO)	
