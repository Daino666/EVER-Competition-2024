import rospy
import numpy as np
from std_msgs.msg import Float32, String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

depth = 0
color = ''
color_list = ['yellow', 'blue']

def emergency_stop():
    cmd_pub.publish(0)
    rospy.sleep(3)

def manage_depth(msg):
    global depth
    depth = msg.data
    #rospy.loginfo(depth)
    
def manage_color(msg):
    global color
    color = msg.data

def init_node():
    global cmd_pub, steering_pub , color_list

    rospy.init_node("TEST", anonymous=True)
    #odom_sub = rospy.Subscriber('/aft_mapped_adjusted', Odometry)
    cmd_pub = rospy.Publisher('/in_Car_velocity_in_KM/H', Float32, queue_size=10)
    steering_pub = rospy.Publisher('/in_Car_steering_in_degree', Float32, queue_size=10)
    rospy.Subscriber('/depth', Float32, manage_depth)
    rospy.Subscriber('/color', String, manage_color)
    rate = rospy.Rate(10)
    

    while not rospy.is_shutdown():
        speed = 3.0  
        steering_angle = 0

        cmd_pub.publish(speed)
        steering_pub.publish(steering_angle)
        
        rospy.loginfo("Publishing: {}".format(depth))
        
        if (depth <= 2700 and depth > 0):
            cmd_pub.publish(2)
            if color == 'yellow':
                steering_pub.publish(15)
                
            elif color == 'blue':
                steering_pub.publish(-15)

        rate.sleep()


if __name__ == '__main__':
    try:
        init_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
