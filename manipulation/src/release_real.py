#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class Release():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()

        

    def go_to_trash(self):
        #Step1: Go towards trash can [ARM GROUP]
        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 1.52
        self.group_variable_values_arm_goal[2] = 0.0
        self.group_variable_values_arm_goal[3] = 0.0
        self.group_variable_values_arm_goal[4] = 0.0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan1 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)

    def open_gripper(self):
        # Step2: pose_goal joint values [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 1.57
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan2 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def retreat(self):
        # Step3: Retreat/Move back [ARM GROUP]
        self.group_variable_values_arm_goal[0] = 0
        self.group_variable_values_arm_goal[1] = 0.45
        self.group_variable_values_arm_goal[2] = -0.82
        self.group_variable_values_arm_goal[3] = 1.95
        self.group_variable_values_arm_goal[4] = 0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan3 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)
        

    def main(self):

        print('1st lets verify the reference frame to set the EE Goal Pose')
        print(self.group_arm.get_pose_reference_frame())

        rospy.loginfo('Go towards trash ..')
        release_object.go_to_trash()
        rospy.loginfo('release coke can..')
        release_object.open_gripper()
        rospy.loginfo('Retreating..')
        release_object.retreat()
        
        rospy.loginfo('Shuting Down ..')
        moveit_commander.roscpp_shutdown()
        
       


if __name__ == '__main__':
    rospy.init_node('release_coke_to_trash', anonymous=True)
    release_object = Release()
    try:
        release_object.main()
    except rospy.ROSInterruptException:
        pass
