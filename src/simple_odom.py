#!/usr/bin/env python
import rospy
from geometry_msgs.msg._Pose import Pose
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from nav_msgs.msg._Odometry import Odometry
from tf import TransformListener

class SimpleOdom(): 

    def __init__(self):
        rospy.init_node('SimpleOdom')
        rospy.loginfo('SimpleOdom node started')

        self.tf = TransformListener()
        self.map_resolution = 0
        self.new_pose = Pose()

        self.odomSub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback)

        #publisher pose
        self.pub_odom_pose = rospy.Publisher('simple_odom_pose', Pose, queue_size=10)

        #rate
        self.rate = rospy.Rate(2)
        rospy.spin()

        
    def _odom_callback(self, odom):
        if self.map_resolution > 0:
            # TODO: this seems not to be correct, robot is somewhere in the nowhere
            # convert from robot coordinates to map coordinates

            #if self.tf.frameExists("base_link") and self.tf.frameExists("map"):
            try:
                t = self.tf.getLatestCommonTime("map", "base_footprint")
                position, quaternion = self.tf.lookupTransform("map", "base_footprint", t)

                self.new_pose.position.x = position[0]
                self.new_pose.position.y = position[1]
                self.new_pose.position.z = position[2]
                self.new_pose.orientation.x = quaternion[0]
                self.new_pose.orientation.y = quaternion[1]
                self.new_pose.orientation.z = quaternion[2]
                self.new_pose.orientation.w = quaternion[3]

                self.pub_odom_pose.publish(self.new_pose)

            except:
                print('transform not ready')
    

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution

if __name__ == '__main__':
	SimpleOdom()


