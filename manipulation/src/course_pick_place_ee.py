#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_place_node_ee_pose', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_gripper = moveit_commander.MoveGroupCommander("gripper")
# display_trajectory_publisher = rospy.Publisher(
#     '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


# pose referenced to base_link: Use TF
# For Manipulation Project: TF from /world
# For Last Project: TF from /pc_cam_base_link

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w =
pose_target.position.x =
pose_target.position.y =
pose_target.position.z =
group_arm.set_pose_target(pose_target)

plan1 = group_arm.plan()
group_arm.go(wait=True)
rospy.sleep(5)

moveit_commander.roscpp_shutdown()

# pose from world frame#########################

# position:
#   x: 5.455440008713199
#   y: -3.8795299984673766
#   z: 0.9423993693271865
# orientation:
#   x: 3.629672572223566e-11
#   y: 4.2990879322299025e-10
#   z: -5.545444254804008e-05
#   w: 0.9999999984624025


# # pose from pc_cam_base_link###################

# position:
#   x: 0.7453752160072327
#   y: 0.38867005705833435
#   z: 0.9250856637954712
# orientation:
#   x: -0.01357284840196371
#   y: -0.008663436397910118
#   z: 0.7067895531654358
#   w: 0.7072405815124512
