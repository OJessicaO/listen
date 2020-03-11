#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class RobotMove:
    #define the constructor of the class
    def  __init__(self):
        #initialize the ROS node with a name person_finder
        rospy.init_node('person_finder')

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.pc_callback)

        #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)

        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        #make sure to make the robot move by default
        self.cmd_vel.linear.x=0;
        self.cmd_vel.angular.z=0;


        rospy.loginfo("Ready to receive point cloud commands")
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()


    def pc_callback(self, msg):
        
        # pc_list = list(pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'), uvs=[(x_center, y_center)]))
        y_total, x_total, z_total, n = 0, 0, 0, 0
        min_x = -2
        max_x = 2
        max_depth = 4

        print(x_total, -y_total, -z_total)
        for point in point_cloud2.read_points(msg, skip_nans=True):
        	x_total += point[2]
    		y_total += 0-point[0]
    		z_total += 0-point[1]
    		n += 1
    	x_total /= n
    	y_total /= n
    	z_total /= n
    	if y_total<max_x and y_total>min_x and x_total < max_depth:
	    	print(n, x_total/n, y_total/n, z_total/n)

        # if n>500:
        # 	print(n, x_total/n, y_total/n, z_total/n)



if __name__=="__main__":
    try:
      RobotMove()
      rospy.spin()
    except rospy.ROSInterruptException:
      rospy.loginfo("Movement terminated.")