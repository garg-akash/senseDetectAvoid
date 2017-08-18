#!/usr/bin/env python

NAME = 'find_roots_server'
from velocityobs.srv import *
import rospy
import numpy as np

def handle_find_roots(req):
    print "Server got values %s,%s,%s,%s,%s,%s"%(req.a, req.b, req.c, req.d, req.e, req.f)
    vec = [req.a, req.b, req.c, req.d, req.e]
    # vec = np.array(vec)
    aa = np.roots(vec)
    ab = 2*np.arctan(aa)
    print "Theta: ",ab,"\n"
    j = 0
    bb = []
    
    for i in range(len(ab)):
        if np.isreal(ab[i]):
            bb.append(ab[i])
            print "Real Theta: ",np.real(ab[i]),"\n"
            j+=1
    print "BB length",len(bb),"\n"
    if j!=0:
        cc = [0]*len(bb)
        for k in range(len(bb)):
            cc[k] = abs(bb[k]-req.f)
        min_theta = min(cc)
        ind = np.argmin(cc)
        if ab[ind]<req.f:
            min_theta = -1*min_theta
        new_theta = min_theta + req.f
        print "New theta: ",new_theta,"\n"
        #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
        return MyRootsResponse(new_theta)
    else:
	print "No real root exist\n"
	return MyRootsResponse(None)

def find_roots_server():
    print "****"
    rospy.init_node(NAME)
    s = rospy.Service('find_roots', MyRoots, handle_find_roots)
    print "Ready to find roots."
    rospy.spin()
 
if __name__ == "__main__":
    print "function called"
    find_roots_server()

