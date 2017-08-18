#!/usr/bin/env python
 
import sys
import rospy
from velocityobs.srv import *


def find_roots_client(t,u,v,x,y,z):
    print "Running till here -1\n"
    rospy.wait_for_service('find_roots')
    print "Running till here 0\n"
    try:
        print "Running till here 1\n"
        find_roots = rospy.ServiceProxy('find_roots', MyRoots)
        resp1 = find_roots(t,u,v,x,y,z)
	print "Running till here 2\n"
        return resp1.g
    except rospy.ServiceException, r:
        print "Service call failed: %s"%r
 
def usage():
    return "%s [t u v x y z]"%sys.argv[0]
 
if __name__ == "__main__":
    if len(sys.argv) == 7:
	print "got values\n"
        t = float(sys.argv[1])
        u = float(sys.argv[2])
	v = float(sys.argv[3])
	x = float(sys.argv[4])
	y = float(sys.argv[5])
	z = float(sys.argv[6])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s,%s,%s,%s,%s,%s"%(t, u,v,x,y,z)
    print "%s,%s,%s,%s,%s,%s = %s"%(t,u,v,x,y,z,find_roots_client(t,u,v,x,y,z))

