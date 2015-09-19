#!/usr/bin/python

"""
@package This package contains the geometry tool to evaluate angle
		 or change of base etc...
"""

from math import *
import numpy as np 

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
@param pts_B : point B
@param pts_A: point A
@return Return the angle in degrees
"""
def angle(pts_A, pts_B ):
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

    return round(yaw,2)


"""
Enable to change base in 2D (relatives --> absolues)
@param xr : x coordonate rom base 2
@param yr:  y coordonate rom base 2
@param angle: angle between base 1 and base 2
@return Return (x,y) coordinate from base 1
"""

#yaw en rad repere relatif --> absolu
def chg_base_abs(xr, yr, angle):

    xa= xr*cos(angle) + yr*sin(angle)
    ya= -xr*sin(angle) + yr*cos(angle)

    return (xa,ya)


"""
	Get distances from two waypoints
"""
def get_dst_2WP(loc1, loc2):
	(lat1, lon1)=loc1
	(lat2, lon2)=loc2

	#Converssion
	d2r=pi/180
	lat1*=d2r
	lat2*=d2r
	lon1*=d2r
	lon2*=d2r

	dLat=lat2 - lat1
	dLon=lon2 - lon1

	a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
	c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
	ground_dist = 6378137.0 * c
	return ground_dist

def PCA(data, dims_rescaled_data=2):
    """
    returns: data transformed in 2 dims/columns + regenerated original data
    pass in: data as 2D NumPy array
    """
    import numpy as NP
    from scipy import linalg as LA
    m, n = data.shape
    # mean center the data
    data -= data.mean(axis=0)
    # calculate the covariance matrix
    R = NP.cov(data, rowvar=False)
    # calculate eigenvectors & eigenvalues of the covariance matrix
    # use 'eigh' rather than 'eig' since R is symmetric, 
    # the performance gain is substantial
    evals, evecs = LA.eigh(R)
    # sort eigenvalue in decreasing order
    idx = NP.argsort(evals)[::-1]
    evecs = evecs[:,idx]
    # sort eigenvectors according to same index
    evals = evals[idx]
    # select the first n eigenvectors (n is desired dimension
    # of rescaled data array, or dims_rescaled_data)
    evecs = evecs[:, :dims_rescaled_data]
    # carry out the transformation on the data using eigenvectors
    # and return the re-scaled data, eigenvalues, and eigenvectors
    return NP.dot(evecs.T, data.T).T, evals, evecs


def angles_mean(angle_mean):

	mean=0
	minagl=min(angle_mean)
	maxagl=max(angle_mean)
	#Compute the angles average
	if minagl<90 and maxagl>270:
		for agl in angle_mean:
			if agl<90 or agl>270:
				agl=(90+agl)%360
				mean+=agl
			else:
				return (True, None)

		mean=(mean/len(angle_mean)-90)%360

	elif (maxagl-minagl)<180:
		mean=np.mean(angle_mean)
	else:
		return (True, None)

	return (False, mean)