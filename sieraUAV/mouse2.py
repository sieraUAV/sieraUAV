#!/usr/bin/python

import numpy as np
import cv2
from cv2 import cv
import time
from tools.test_kalman import *
from kalman_tool import kalman2D

meas=[]
pred=[]
frame = np.zeros((1000,1000,3), np.uint8) # drawing canvas
mp = np.array((2,1), np.float32) # measurement
tp = np.zeros((2,1), np.float32) # tracked / prediction

def onmouse(k,x,y,s,p):
    global mp,meas
    mp = np.array([[np.float32(x)],[np.float32(y)]])
    meas.append((x,y))

def paint():
    global frame,meas,pred
    for i in range(len(meas)-1): cv2.line(frame,meas[i],meas[i+1],(0,100,0))
    for i in range(len(pred)-1): cv2.line(frame,pred[i],pred[i+1],(0,0,200))

def reset():
    global meas,pred,frame
    meas=[]
    pred=[]
    frame = np.zeros((1000,1000,3), np.uint8)

cv2.namedWindow("kalman")
cv2.setMouseCallback("kalman",onmouse);
cv2.imshow("kalman",frame)

while mp.item(0)<=0 or mp.item(1)<=0:
    print mp


mouseK= kalman2D(mp.item(0), mp.item(1))


while True:

    mouseK.predict()

    x,y= mouseK.correct(mp.item(0), mp.item(1))

    #x, P = kalman_xy(x, P, (mp.item(0), mp.item(1)) , R, Q = np.matrix(np.eye(4))*1000)

    pred.append((x, y))
    paint()
    cv2.imshow("kalman",frame)
    k = cv2.waitKey(30) &0xFF
    if k == 27: break
    if k == 114: reset()