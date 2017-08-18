import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

def callback(data):
    pub = rospy.Publisher('self/azimuth', Float64, queue_size=1)
    vec = [data[0], data[1], data[2], data[3], data[4]]
    aa = np.roots(vec)
    a = 2*np.arctan(aa)
    print("Theta: ",a,"\n")
    j = 0
    for i in range(len(a)):
        if np.isreal(a[i]):
            b[j] = a[i]
            print("Real Theta: ",np.real(a[i]),"\n")
            j++
    for k in range(len(b)):
        c[k] = abs(b[k]-data[5])
    min_theta = min(c)
    ind = np.argmin(c)
    if a[ind]<data[5]:
        min_theta = -1*min_theta
    new_theta = min_theta + data.angular.z
    print("New theta: ",new_theta,"\n")
    pub.publish(Float64(new_theta))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('self/eqcoeff', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()