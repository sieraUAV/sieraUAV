#!/usr/bin/python
import cv2
import time
import Image
import math
from math import pi
import numpy as np
import threading
import Queue



exitFlag = 0
QueueVideo=Queue.Queue(10)

#THREAD CLASS DEF
class myThread (threading.Thread):
    def __init__(self, threadID, name, function):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.function = function
    def run(self):
        print "Starting " + self.name
        self.function()
        print "EXIT " + self.name

 
#FONCTION TOUCHE
def toucheAction(delai=10):
     
    flag=False   
    """
    numero de touche
    's'--> sauvgarde une image
    'ESC' --> quitte le programe
    'g' --> demarre ou arrete la capture
    """
<<<<<<< HEAD
    save=115
    esp=27
    video=118
     
=======
    #conf 1
    #save=1048691
    #esp=1048603
    #video=1048679

    #conf 2
    save=115
    esp=27
    video=118

>>>>>>> 44e05713f972f39d512c59280b58a2a75bc9e960
    k = cv2.waitKey(delai)
    
    if k!=-1:
        print 'touche:',k
    
    if k==save:
        flag='save'
    elif k == esp:         # wait for ESC key to exit
        flag='esc'
    elif k == video: # wait for 's' key to save and exit
        flag='video'
    else:
        flag=False
    return flag

def interdistance(bary_Nth,w,h):
    (x1,y1) = bary_Nth
    (x2,y2) = (int(w/2),int(h/2))
    distancex= x1-x2
    distancey= y2-y1
    distances=(distancex, distancey)
    return distances
 
def angle(bary_Nth, bary_Sth):
    (x1,y1)=bary_Nth
    (x2,y2)=bary_Sth
    deltaX=x1-x2
    deltaY=y2-y1
    convert=float(180)/pi

    yaw=0

    if deltaX!=0 and deltaY!=0:
        if deltaX>0:
            if deltaY>0:
                yaw=math.atan(float(deltaX)/deltaY)*convert
            else:
                yaw=(pi-math.atan(float(deltaX)/(abs(deltaY))))*convert
        else:
            if deltaY>0:
                yaw=(2*pi-math.atan(float(abs(deltaX))/deltaY))*convert
            else:
                yaw=(pi+math.atan(float(abs(deltaX))/(abs(deltaY))))*convert
    elif deltaY==0 and deltaX!=0:
        if deltaX<0:
            yaw=3*pi/2*convert
        else:
            yaw=pi/2*convert
    elif deltaY!=0 and deltaX==0:
        if deltaY<0:
            yaw=pi*convert
        else:
            yaw=0
    else:
        print "non detection" 

    return int(yaw)
     

"""
IMAGE PROSSESSING
"""

def videoThread():
    #INIT WINDOW
    cv2.namedWindow("Composed", cv2.CV_WINDOW_AUTOSIZE)
    cv2.namedWindow("img", cv2.CV_WINDOW_AUTOSIZE)
    cv2.namedWindow("imageHSV_hist", cv2.CV_WINDOW_AUTOSIZE)
    cv2.startWindowThread()

    #CONFIG CAPTURE
    capture=cv2.VideoCapture(0)
     
    #CONFIG CAPTURE VIDEO
    w=int(capture.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH ))
    h=int(capture.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT ))
    print 'hauteur %d et largeur %d' % (h ,w)
    out = cv2.VideoWriter('output.avi',cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 10,(2*w , h),1)
     
    #INITIALISATION DES BOOL
    boolBoucle=True
    boolVideo=False


    while (boolBoucle):
        #CAPTURE
        capture.read()
        ret, img = capture.read()
        rows,cols,channels = img.shape

         
        #TRAITEMENT VIDEO
        """
        # separer les couleurs
        b,g,r = cv2.split(img)
        # egaliser l'hisograme sur chaque couleur
        b=cv2.equalizeHist(b)
        g=cv2.equalizeHist(g)
        r=cv2.equalizeHist(r)
        # refusioner l'image
        img = cv2.merge((b,g,r))
        """
             
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
<<<<<<< HEAD
        min_red = np.array((0. ,100. ,60. ))
        max_red = np.array((7. ,255. ,255. ))
=======
        min_red = np.array((0. ,80. ,60. ))
        max_red = np.array((12. ,255. ,255. ))
>>>>>>> 44e05713f972f39d512c59280b58a2a75bc9e960
         
        min_red2 = np.array((170. ,80. ,60. ))
        max_red2 = np.array((180. ,255. ,255. ))
     
        imgThresh = cv2.inRange(imageHSV, min_red, max_red)
        imgThresh2= cv2.inRange(imageHSV, min_red2, max_red2)
        imgThreshT=cv2.bitwise_or(imgThresh,imgThresh2)
     
        #close function (effacer les trous)
        kernel = np.ones((7,7),np.uint8)
        closing = cv2.morphologyEx(imgThreshT, cv2.MORPH_CLOSE, kernel)
         
        """
        #Canny
        canny = dst = cv2.Canny( blur, 0, 255 )
        """
         
        #repere
        pt1=(cols/2,0)
        pt2=(cols/2,rows)
        cv2.line(img,pt1,pt2,[255,255,255],1)
        pt3=(0,rows/2)
        pt4=(cols,rows/2)
        cv2.line(img,pt3,pt4,[255,255,255],1)

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
                        centre=(int(w*0.5),int(h*0.5))








        #trie des contour filtre par aire les plus grandre
        cnt_filt.sort(reverse=True)

        #print cnt_filt
        if len(cnt_filt)>0:
            cv2.circle(img,cnt_filt[0][2],4,(255,0,0),-1)

            (a,b)=cnt_filt[0][2]
            ptx=(a,int(h*0.5))
            pty=(int(w*0.5),b)
            cv2.line(img,cnt_filt[0][2],ptx,[0,0,255],1)
            cv2.line(img,cnt_filt[0][2],pty,[0,0,255],1)

            yaw=angle(cnt_filt[0][1], cnt_filt[0][2])



            distances=interdistance(cnt_filt[0][1],w,h)
            #print distances, yaw
            cv2.drawContours(img,[cnt_filt[0][3]],0,(0,255,0),2)
            


            if QueueVideo.full():
                QueueVideo.get()
            #Put item in queue
            sendItem= ('DETECT',distances,yaw)
            QueueVideo.put(sendItem)
                     
         
        #TEST ACTION
        act=toucheAction(20)
        if act:
            print 'Action:',act
            if act=='save':
                cv2.imwrite('capture.png',img)
            elif act=='esc':
                boolBoucle=False
            elif act=='video':
                boolVideo= not boolVideo
                if boolVideo:
                    print 'video: REC'
                else:
                    print 'video: PAUSE'
                     
        #CREATE COMPOSED IMAGE
        rows,cols,channels = img.shape
        compoImage = np.zeros((rows,2*cols,3), np.uint8)
        compoImage[0:rows, 0:cols ] = img
        imgThreshTRGB=cv2.cvtColor ( imgThreshT, cv2.COLOR_GRAY2BGR );
        compoImage[0:rows, cols:2*cols ] = imgThreshTRGB
        #CAPTURE VIDEO
        if boolVideo:
            out.write(compoImage)
         
        #AFFICHAGE# separer les couleurs
        cv2.imshow("imageHSV_hist", imageHSV_hist)
        #cv2.imshow("imgThresh", imgThresh)
        #cv2.imshow("imgThresh2", imgThresh2)
        #cv2.imshow("imgThreshT", imgThreshT)
        #cv2.imshow("cont", cont)
        #cv2.imshow("canny", canny)
        cv2.imshow("img", img)
        #cv2.imshow("closing", closing)
        cv2.imshow("Composed", compoImage)
     
    capture.release()
    cv2.destroyAllWindows()
