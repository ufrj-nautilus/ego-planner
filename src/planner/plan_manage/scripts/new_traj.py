#!/usr/bin/env python 

print("INITIALIZING NEW_TRAJ")

# Imports
import rospy
from geometry_msgs.msg import Transform
from ego_planner.msg import Bspline
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform

class BsplineConverter:
    def __init__(self):
      ego_planner_traj_topic = rospy.get_param('~ego_planner_traj_topic', '/planning/bspline')

      # Publisher for UAV position control
      traj_pub_topic = rospy.get_param('~traj_pub_topic', '/red/tracker/input_trajectory')

      self.counter = 0
      
      # Subscriber for Ego-Planner reference trajectory
      rospy.Subscriber(ego_planner_traj_topic, Bspline, callback=self.traj_callback)

      self.pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=10)
  
    def traj_callback(self, msg):
      multi_dof_trajectory = MultiDOFJointTrajectory()

      print("----------NUMBER OF CALLBACKS-------------", self.counter)

      print("----------LENGTH MSG.POS_PTS----------", len(msg.pos_pts))
      if self.counter == 0:
        for i in range(len(msg.pos_pts)):
          temp_point = MultiDOFJointTrajectoryPoint()
          temp_transform = Transform()
          temp_transform.translation.x = msg.pos_pts[i].x
          temp_transform.translation.y = msg.pos_pts[i].y
          temp_transform.translation.z = msg.pos_pts[i].z
          temp_transform.rotation.w = 1.0

          temp_point.transforms.append(temp_transform)
          multi_dof_trajectory.points.append(temp_point)

        self.counter += 1
        self.pub.publish(multi_dof_trajectory)

if __name__ == '__main__':
    rospy.init_node('bspline_msg_converter')
    print("INIT NODE")
    obj = BsplineConverter()
    rospy.spin()

""" rostopic echo -n1 /planning/bspline - 

Type: ego_planner/Bspline

Publishers: 
 * /ego_planner_node (http://lucca-SVF15213CBW:33761/)

Subscribers: 
 * /traj_server (http://lucca-SVF15213CBW:43621/)


order: 3
traj_id: 1
start_time: 
  secs: 3358
  nsecs: 179000000
knots: [-0.5120709584384695, -0.3413806389589797, -0.17069031947948984, 0.0, 0.17069031947948984, 0.3413806389589797, 0.5120709584384695, 0.6827612779179594, 0.8534515973974492, 1.024141916876939, 1.1948322363564288, 1.3655225558359185, 1.5362128753154083, 1.706903194794898, 1.8775935142743878, 2.0482838337538776]
pos_pts: 
  - 
    x: -9.992398693109015
    y: 0.013144624961962463
    z: 0.542364572647488
  - 
    x: -9.9922284266492
    y: 0.013439651271538181
    z: 0.5424169259163675
  - 
    x: -9.992056722245646
    y: 0.013737169286292502
    z: 0.5424697213769454
  - 
    x: -9.934990870065192
    y: 0.11777821581521175
    z: 0.5615226333884763
  - 
    x: -9.806907339922425
    y: 0.32022073369783033
    z: 0.6081696456512161
  - 
    x: -9.620312771805821
    y: 0.614886259402226
    z: 0.6806909701645365
  - 
    x: -9.403544037976967
    y: 0.9949890230437746
    z: 0.7682413418313626
  - 
    x: -9.188916623549845
    y: 1.3732977597632217
    z: 0.8552829970842278
  - 
    x: -9.004042898005176
    y: 1.6660749130527681
    z: 0.9286413514044982
  - 
    x: -8.868805794646896
    y: 1.8665415007017852
    z: 0.9807512839957077
  - 
    x: -8.807528660448924
    y: 1.9677008047990951
    z: 1.0045883876850945
  - 
    x: -8.868635952505063
    y: 1.866850057196867
    z: 0.980794827258877
yaw_pts: []
yaw_dt: 0.0
---"""