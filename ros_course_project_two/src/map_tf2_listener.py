#!/usr/bin/env python
import rospy
import tf2_ros

if __name__ == '__main__':
    #Initializes a listener node to display the robot position. 
    rospy.init_node('map_tf2_listener')

    #Creates a buffer object
    tfBuffer = tf2_ros.Buffer()
    #Creates a transform listener using the buffer above. 
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0) #Sets up a transmission rate. 

    while not rospy.is_shutdown():

        try:
            #Gets the transformation matrix between the base of the robot and the map. 
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #Error handling. 
            rospy.logerr("An error occurred, please try again. ") 
            rate.sleep()
            continue

        rospy.loginfo(trans.transform) #Logs the transformation data (both rotation and translation values). 

        rate.sleep()
