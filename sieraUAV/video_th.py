#!/usr/bin/python
import cv2
import time
import Image
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
def toucheAction(delai=50):
     
    flag=False   
    """
    numero de touche
    's'--> sauvgarde une image
    'ESC' --> quitte le programe
    'g' --> demarre ou arrete la capture
    """
    save=1048691
    esp=1048603
    video=1048679
     
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
 
"""
IMAGE PROSSESSING
"""

def videoThread():
    #INIT WINDOW
    cv2.namedWindow("Composed", cv2.CV_WINDOW_AUTOSIZE)
    cv2.startWindowThread()

    #CONFIG CAPTURE
    capture=cv2.VideoCapture(1)
     
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
     
        #thresholding
        min_red = np.array((0. ,100. ,60. ))
        max_red = np.array((12. ,255. ,255. ))
         
        min_red2 = np.array((170. ,100. ,60. ))
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
         
        #contour
        contours,hier = cv2.findContours(closing,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
     
        #Filtrage de contour
        #init list element filtre
        cnt_filt=[]
        for cnt in contours:
            CurrAera=cv2.contourArea(cnt)
            if CurrAera>300 :  # remove small areas like noise etc
                #hull = cv2.convexHull(cnt)    # find the convex hull of contour
                hull = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
                #if len(hull)==12:
                if not cv2.isContourConvex(hull) and len(hull)<=12:
                    m = cv2.moments(hull)
                    if m['m00'] !=0:
                        center=(int(m['m10']/m['m00']),int(m['m01']/m['m00']))
                        #register info 
                        cnt_filt.append((CurrAera, center, hull))
                        cv2.drawContours(img,[hull],0,(0,0,255),2)

        #trie des contour filtre par aire les plus grandre
        cnt_filt.sort(reverse=True)

        #print cnt_filt
        if len(cnt_filt)>0:
            cv2.circle(img,cnt_filt[0][1],4,(255,0,0),-1)                
            cv2.drawContours(img,[cnt_filt[0][2]],0,(0,255,0),2)

            if QueueVideo.full():
                QueueVideo.get()
            #Put item in queue
            QueueVideo.put('DETECT')
                     
         
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
        #cv2.imshow("imageHSV", imageHSV)
        #cv2.imshow("imgThresh", imgThresh)
        #cv2.imshow("imgThresh2", imgThresh2)
        #cv2.imshow("imgThreshT", imgThreshT)
        #cv2.imshow("cont", cont)
        #cv2.imshow("canny", canny)
        #cv2.imshow("img", img)
        #cv2.imshow("closing", closing)
        cv2.imshow("Composed", compoImage)
     
    capture.release()
    cv2.destroyAllWindows()
