#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class Orient_Camera():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()

 
    def perception_pose(self):
        # Step1: Orient Camera in order to launch our Perception node [ARM GROUP]

        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 0.4
        self.group_variable_values_arm_goal[2] = 0.0
        self.group_variable_values_arm_goal[3] = 2
        self.group_variable_values_arm_goal[4] = 0.0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)


    def main(self):

        print('1st lets verify the reference frame to set the EE Goal Pose')
        print(self.group_arm.get_pose_reference_frame())

        rospy.loginfo('Orient Camera towards the table ..')
        orient_camera_object.perception_pose()

        rospy.loginfo('Shuting Down ..')
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('orient_camera_node', anonymous=True)
    orient_camera_object = Orient_Camera()
    try:
        orient_camera_object.main()
    except rospy.ROSInterruptException:
        pass
