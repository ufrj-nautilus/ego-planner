#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Ego-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by TOPP-RA
Author: Mohamed Abdelkader
Modifications: Lucca Gandra
"""

# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for TOPP-RA
from quadrotor_msgs.msg import PositionCommand # for Ego-Planner
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'red/tracker/input_trajectory')

        # Publisher for TOPP-RA
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.egoPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def egoPlannerTrajCallback(self, msg):
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)

if __name__ == '__main__':
    obj = MessageConverter()