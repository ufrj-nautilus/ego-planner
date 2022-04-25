#!/usr/bin/env python 

# Imports
import rospy
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Bool

class PositionCommandConverter:
    def __init__(self):
      rospy.init_node('pos_command_msg_converter')

      ego_planner_traj_topic = rospy.get_param('~ego_planner_traj_topic', 'planning/pos_cmd')

      self.counter = 0
      self.kill = False
      
      # Publisher for UAV position control
      traj_pub_topic = rospy.get_param('~traj_pub_topic', '/red/position_hold/trajectory')

      # Subscriber for Ego-Planner reference trajectory
      rospy.Subscriber(ego_planner_traj_topic, PositionCommand, callback=self.traj_callback)

      self.pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectoryPoint, queue_size=10)

      rospy.Subscriber("/red/kill_traj_planner", Bool, callback=self.kill_callback)

      rospy.spin()
  
    def traj_callback(self, msg):
      # PositionCommand --> MultiDOFJointTrajectoryPoint
      traj_point = MultiDOFJointTrajectoryPoint()

      self.counter += 1
    
      pose = Transform()
      pose.translation.x = msg.position.x
      pose.translation.y = msg.position.y
      pose.translation.z = msg.position.z
      quaternion = quaternion_from_euler(0, 0, msg.yaw) 
      pose.rotation.x = quaternion[0]
      pose.rotation.y = quaternion[1]
      pose.rotation.z = quaternion[2]
      pose.rotation.w = quaternion[3]

      vel = Twist()
      acc = Twist()

      traj_point.transforms.append(pose)
      traj_point.velocities.append(vel)
      traj_point.accelerations.append(acc)
      
      traj_point.time_from_start.secs = 0
      traj_point.time_from_start.nsecs = 0

      #print("Counter: ", self.counter)
      #print("Msg.position.x: ", msg.position.x)

      if(self.kill == False):
        self.pub.publish(traj_point)

    def kill_callback(self, msg):
        if msg.data == True:
          self.kill = True
        else:
          self.kill = False      

if __name__ == '__main__':
    obj = PositionCommandConverter()

""" rostopic echo -n1 /planning/pos_cmd - 

Type: PositionCommand

header: 
  seq: 998
  stamp: 
    secs: 1636749300
    nsecs: 458358945
  frame_id: "world"
position: 
  x: -9.28099878117878
  y: -8.688725040383979
  z: 0.9997694852660133
velocity: 
  x: 0.0
  y: 0.0
  z: 0.0
acceleration: 
  x: 0.0
  y: 0.0
  z: 0.0
yaw: -1.364799524977911
yaw_dot: 0.0
kx: [5.7, 5.7, 6.2]
kv: [3.4, 3.4, 4.0]
trajectory_id: 9
trajectory_flag: 1
---"""