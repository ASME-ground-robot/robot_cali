#! /usr/bin/env python

import sys
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose


class Pick_Place_EE_Pose():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # # Set the reference frame for pose targets
        # reference_frame = "scorbot_base_link"
        # # Set the scorbot_er3 reference frame accordingly
        # self.group_arm.set_pose_reference_frame(reference_frame)

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()
        self.pose_target = Pose()
        self.rate = rospy.Rate(10)

        self.offset_x = 0.00
        self.offset_y = 0.00
        self.offset_z = 0.08

    def get_data(self):
        # self.group_arm.set_end_effector_link("end_effector_link")
        self.group_arm.set_goal_position_tolerance(0.05)
        self.group_arm.set_goal_orientation_tolerance(0.01)
        self.group_arm.set_goal_tolerance(0.01)
    
        print("Reference frame: %s" %
      self.group_arm.get_planning_frame())  # = world = base_link

        print("End effector: %s" % self.group_arm.get_end_effector_link())  # = wrist_3_link

        print("Robot Groups:")
        print(self.robot.get_group_names())

        print("Current Joint Values:")
        print(self.group_arm.get_current_joint_values())

        print("Current Pose:")
        print(self.group_arm.get_current_pose())

        print("Robot State:")
        print(self.robot.get_current_state())
    
    
    def open_gripper(self):
        # Step1: Open Gripper joint value [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 1.57
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan1 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def pregrasp(self):

        

        self.pose_target.position.x = 0.702
        self.pose_target.position.y = 0.039
        self.pose_target.position.z = 0.821

        self.pose_target.orientation.x = -0.002
        self.pose_target.orientation.y = 0.04
        self.pose_target.orientation.z = 0.045
        self.pose_target.orientation.w = 0.998

        

        # self.pose_target.position.x = 0.74417257309- self.offset_x
        # self.pose_target.position.y = 0.0232634060085
        # self.pose_target.position.z = 0.787107050419 + self.offset_z

        # self.pose_target.orientation.x = 0.00
        # self.pose_target.orientation.y = 0.0
        # self.pose_target.orientation.z = 0.0
        # self.pose_target.orientation.w = 1

        # rospy.logerr("LETS PRINT OUR GOAL POSE FROM THE BASE_LINK OF THE ARM:")
        # rospy.logerr(self.pose_target.position.x)
        # rospy.logerr(self.pose_target.position.y)
        # rospy.logerr(self.pose_target.position.z)
        # print(self.pose_target.position.x)
        # print(self.pose_target.position.y)
        # print(self.pose_target.position.z)
        self.group_arm.set_pose_target(self.pose_target)

        self.plan2 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        # print("CONFIRMIIIIIIIIIIIING!!!!!!!!")
        # rospy.logerr(self.pose_target.position.x)
        # rospy.logerr(self.pose_target.position.y)
        # rospy.logerr(self.pose_target.position.z)
        # print(self.pose_target.position.x)
        # print(self.pose_target.position.y)
        # print(self.pose_target.position.z)
        rospy.sleep(2)

    # def grasp(self):
    #     # Step2: Close Gripper joint value [GRIPPER GROUP]

    #     self.group_variable_values_gripper_close[0] = -0.7
    #     self.group_gripper.set_joint_value_target(
    #         self.group_variable_values_gripper_close)

    #     self.plan2 = self.group_gripper.plan()

    #     self.group_gripper.go(wait=True)
    #     rospy.sleep(2)

    # def retreat(self):
    #     # Step3: Retreat/Move back [ARM GROUP]

    #     self.group_variable_values_arm_goal[0] = 0
    #     self.group_variable_values_arm_goal[1] = -1.7
    #     self.group_variable_values_arm_goal[2] = 1.43
    #     self.group_variable_values_arm_goal[3] = -1.76
    #     self.group_variable_values_arm_goal[4] = -1.51
    #     self.group_variable_values_arm_goal[5] = -1.6298
    #     self.group_arm.set_joint_value_target(
    #         self.group_variable_values_arm_goal)

    #     self.plan3 = self.group_arm.plan()

    #     self.group_arm.go(wait=True)
    #     rospy.sleep(2)

    # def rotate(self):
    #     # Step4: Turn Shoulder Joint 180 degrees [ARM GROUP]

    #     self.group_variable_values_arm_goal[0] = -3.14
    #     self.group_variable_values_arm_goal[1] = -1.0
    #     self.group_variable_values_arm_goal[2] = 1.43
    #     self.group_variable_values_arm_goal[3] = -1.76
    #     self.group_variable_values_arm_goal[4] = -1.51
    #     self.group_variable_values_arm_goal[5] = -1.6298
    #     self.group_arm.set_joint_value_target(
    #         self.group_variable_values_arm_goal)

    #     self.plan4 = self.group_arm.plan()

    #     self.group_arm.go(wait=True)
    #     rospy.sleep(2)

    # def release_object(self):
    #     # Step5: Open Gripper [GRIPPER GROUP]

    #     self.group_variable_values_gripper_close[0] = 0.2
    #     self.group_gripper.set_joint_value_target(
    #         self.group_variable_values_gripper_close)

    #     self.plan5 = self.group_gripper.plan()

    #     self.group_gripper.go(wait=True)
    #     rospy.sleep(2)

    def main(self):
        
        rospy.loginfo('Getting Data..')
        pick_place_object.get_data()
        rospy.loginfo('Opening Gripper..')
        pick_place_object.open_gripper()
        rospy.loginfo('Pregrasp..')
        pick_place_object.pregrasp()
        # rospy.loginfo('Grasp..')
        # pick_place_object.grasp()
        # rospy.loginfo('Retreating..')
        # pick_place_object.retreat()
        # rospy.loginfo('Rotating..')
        # pick_place_object.rotate()
        # rospy.loginfo('Releasing object..')
        # pick_place_object.release_object()

        rospy.loginfo('Shuting Down ..')
        moveit_commander.roscpp_shutdown()
    


if __name__ == '__main__':
    rospy.init_node('pick_place_ee_node', anonymous=True)
    pick_place_object = Pick_Place_EE_Pose()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass



