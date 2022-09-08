#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class Pick_Place():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()

    def open_gripper(self):
        # Step1: pose_goal joint values [ARM GROUP]

        self.group_variable_values_gripper_close[0] = 1.57
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan1 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)
        

    def pregrasp(self):
        # Step2: pose_goal joint values [ARM GROUP]

        # self.group_variable_values_arm_goal[0] = 0.08
        # self.group_variable_values_arm_goal[1] = 1.15
        # self.group_variable_values_arm_goal[2] = 0.0
        # self.group_variable_values_arm_goal[3] = 0.41
        # self.group_variable_values_arm_goal[4] = 0.0

        self.group_variable_values_arm_goal[0] = 0.09
        self.group_variable_values_arm_goal[1] = 0.33
        self.group_variable_values_arm_goal[2] = 1.32
        self.group_variable_values_arm_goal[3] = 0.0
        self.group_variable_values_arm_goal[4] = 0.0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan2 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)

    def grasp(self):
        # Step3: Close Gripper joint value [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 0.55
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan3 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def retreat(self):
        # Step4: Retreat/Move back [ARM GROUP]

        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 0.45
        self.group_variable_values_arm_goal[2] = -0.82
        self.group_variable_values_arm_goal[3] = 1.95
        self.group_variable_values_arm_goal[4] = 0.0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan4 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)

    # def rotate(self):
    #     # Step5: Turn Shoulder Joint 180 degrees [ARM GROUP]

    #     self.group_variable_values_arm_goal[0] = -3.14
    #     self.group_variable_values_arm_goal[1] = -1.0
    #     self.group_variable_values_arm_goal[2] = 1.43
    #     self.group_variable_values_arm_goal[3] = -1.76
    #     self.group_variable_values_arm_goal[4] = -1.51
    #     self.group_variable_values_arm_goal[5] = -1.6298
    #     self.group_arm.set_joint_value_target(
    #         self.group_variable_values_arm_goal)

    #     self.plan5 = self.group_arm.plan()

    #     self.group_arm.go(wait=True)
    #     rospy.sleep(2)

    # def release_object(self):
    #     # Step6: Open Gripper [GRIPPER GROUP]

    #     self.group_variable_values_gripper_close[0] = 0.2
    #     self.group_gripper.set_joint_value_target(
    #         self.group_variable_values_gripper_close)

    #     self.plan6 = self.group_gripper.plan()

    #     self.group_gripper.go(wait=True)
    #     rospy.sleep(2)

    def main(self):

        print('1st lets verify the reference frame to set the EE Goal Pose')
        print(self.group_arm.get_pose_reference_frame())

        rospy.loginfo('Open Gripper ..')
        pick_place_object.open_gripper()
        rospy.loginfo('Going to Goal Pose..')
        pick_place_object.pregrasp()
        rospy.loginfo('Now Going to Grasp the object..')
        pick_place_object.grasp()
        rospy.loginfo('Retreating..')
        pick_place_object.retreat()
        # rospy.loginfo('Rotating..')
        # pick_place_object.rotate()
        # rospy.loginfo('Releasing object..')
        # pick_place_object.release_object()

        rospy.loginfo('Shuting Down ..')
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('pick_place_node_joint_values', anonymous=True)
    pick_place_object = Pick_Place()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
