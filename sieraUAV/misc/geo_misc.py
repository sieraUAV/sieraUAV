#!/usr/bin/python

"""
@package This package contains the geometry tool to evaluate angle
		 or change of base etc...
"""

from math import *

"""
Enable to get distances (x,y) between a point and the center of an image
@param pts : point from witch get distances
@param w: width of the image
@param h: height of the image
@return Return a distance vector (dist_x, dist_y)
"""
def interdistance(pts,w,h):
    (x1,y1) = pts
    (x2,y2) = (int(w/2),int(h/2))
    distancex= x1-x2
    distancey= y2-y1
    distances=(distancex, distancey)
    return distances
 
"""
Enable to get angle from a vector (2 points: (A,B)) and the base of the image
@param pts_B : point A
@param pts_A: point B
@return Return the angle in degrees
"""
def angle(pts_B, pts_A):
    (x1,y1)=pts_B
    (x2,y2)=pts_A
    deltaX=x1-x2
    deltaY=y2-y1	#inversion du repere de l'image
    convert=float(180)/pi

    yaw=0

    if deltaX!=0 and deltaY!=0:
        if deltaX>0:
            if deltaY>0:
                yaw=atan(float(deltaX)/deltaY)*convert
            else:
                yaw=(pi-atan(float(deltaX)/(abs(deltaY))))*convert
        else:
            if deltaY>0:
                yaw=(2*pi-atan(float(abs(deltaX))/deltaY))*convert
            else:
                yaw=(pi+atan(float(abs(deltaX))/(abs(deltaY))))*convert
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