#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.msg import Odometry
from simple_odom.msg import PoseConverted, CustomPose

class SimpleOdom(): 

    def __init__(self):
        rospy.init_node('SimpleOdom')
        rospy.loginfo('SimpleOdom node started')

        self.map_info = MapMetaData()

        self.tfl = tf.TransformListener()

        self.ready = False

        # --- Publisher ---
        self.pub_odom_pose = rospy.Publisher('simple_odom_pose', CustomPose, queue_size=10)

        # --- Subscribers ---
        self.odomSub = rospy.Subscriber('/odom', Odometry, self._handle_odom)

        self._get_map_meta()
        
        self.seq = 0

        #rate
        self.rate = rospy.Rate(2)
        rospy.spin()

        
    def _handle_odom(self, odom):
        """
        Process information publised to the odom topic.
        """
        try:
            t = self.tfl.getLatestCommonTime("map", "base_footprint")
            position, quaternion = self.tfl.lookupTransform("map", "base_footprint", t)

            # store the pose
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            # calculate pose as map coordinates (row, col)
            pose_converted = PoseConverted()
            pose_converted.x = int(math.floor((pose.position.x - self.map_info.origin.position.x)/self.map_info.resolution))
            pose_converted.y = int(math.floor((pose.position.y - self.map_info.origin.position.y)/self.map_info.resolution))
            pose_converted.yaw = self._robot_angle(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

            customPose = CustomPose()
            customPose.header.seq = self.seq
            customPose.header.stamp = rospy.Time.now()
            customPose.pose = pose
            customPose.pose_converted = pose_converted

            self.pub_odom_pose.publish(customPose)

            if not self.ready:
                print('---- ready ----')
                self.ready = True

        except tf.Exception:
            if self.ready:
                self.ready = False
            print('transform not ready')

    def _get_map_meta(self):
        """
        Get resulution and the offsets
        """
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_info = map.info

    def _robot_angle(self, x, y, z, w):
        """
        Calculate the angle of the robot

        Returns:
        yaw: angle in radiant
        """
        orientation_list = [x, y, z, w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

if __name__ == '__main__':
	try:
        SimpleOdom()
    except:
		pass


