#! /usr/bin/env python
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import rospy
import copy
import sys


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group.set_end_effector_link("end_effector_link")
# group.set_end_effector_link("scorbot_link_4")

print("Reference frame: %s" %
      group.get_planning_frame())  # = world = base_link

print("End effector: %s" % group.get_end_effector_link())  # = wrist_3_link

print("Robot Groups:")
print(robot.get_group_names())

print("Current Joint Values:")
print(group.get_current_joint_values())

print("Current Pose:")
print(group.get_current_pose())

print("Robot State:")
print(robot.get_current_state())

moveit_commander.roscpp_shutdown()
