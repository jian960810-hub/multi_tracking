import rospy
from sensor_msgs.msg import LaserScan
import os

path = 'laser.txt'
f = open(path, 'w')

def leave(data):
    print('hi')
    os._exit(0)

def callback(data):
    print('zz')
    for i in range(360):
        rad = data.angle_min+data.angle_increment*i
        ranges = data.ranges
        #print(rad,ranges[i])
        lines = [str(rad),'    ',str(ranges[i]),'\n']
        f.writelines(lines)
        #rospy.loginfo(data.intensities)
    


def listener():
    rospy.init_node('listener', anonymous=True)   
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Timer(rospy.Duration(360),leave)
    rospy.spin()

if __name__ == '__main__':
    listener()


