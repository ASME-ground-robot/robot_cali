#! /usr/bin/env python

import sys
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from project_object_detection.object_data_extract_node import ObjectFilter


class Pick_Place_EE_Pose():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()
        self.pose_target = Pose()

        self.sub = rospy.Subscriber(
            '/graspable_object_pose', Pose, self.pose_callback)

        self._listener = tf.TransformListener()
        self.rate = rospy.Rate(10)

        # self.pose_x_from_cam = 0.0
        # self.pose_y_from_cam = 0.0
        # self.pose_z_from_cam = 0.0
        # self.pose_x_from_base_link_to_cam = 0
        # self.pose_y_from_base_link_to_cam = 0
        # self.pose_z_from_base_link_to_cam = 0
        self.offset = 0.305

        self.trigger = False

    def pose_callback(self, msg):
        # This is the pose from the camera and we need to convert it into the base_link frame
        # Thats why x direction becomes -y and y position becomes x
        self.pose_x_from_cam = msg.position.y
        self.pose_y_from_cam = msg.position.x
        self.pose_z_from_cam = msg.position.z

    def listener(self):

        rospy.loginfo(
            "We are first listening to the TF from base_link to pc_cam_base_link...")
        self._listener.waitForTransform(
            "base_link", "pc_cam_base_link", rospy.Time(0), rospy.Duration(120))
        (trans, rot) = self._listener.lookupTransform(
            "base_link", "pc_cam_base_link", rospy.Time(0))

        self.pose_x_from_base_link_to_cam = trans[0]
        self.pose_y_from_base_link_to_cam = trans[1]
        self.pose_z_from_base_link_to_cam = trans[2]

    def pregrasp(self):

        self.pose_target.position.x = self.pose_x_from_base_link_to_cam - self.pose_x_from_cam
        self.pose_target.position.y = self.pose_y_from_base_link_to_cam + \
            self.pose_y_from_cam
        self.pose_target.position.z = self.pose_z_from_base_link_to_cam + \
            self.pose_z_from_cam + self.offset
        self.pose_target.orientation.x = -0.685
        self.pose_target.orientation.y = -0.014
        self.pose_target.orientation.z = 0.027
        self.pose_target.orientation.w = 0.728

        rospy.logerr("LETS PRINT OUR GOAL POSE FROM THE BASE_LINK:")
        rospy.logerr(self.pose_target.position.x)
        rospy.logerr(self.pose_target.position.y)
        rospy.logerr(self.pose_target.position.z)
        print(self.pose_target.position.x)
        print(self.pose_target.position.y)
        print(self.pose_target.position.z)
        self.group_arm.set_pose_target(self.pose_target)

        self.plan1 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        print("CONFIRMIIIIIIIIIIIING!!!!!!!!")
        rospy.logerr(self.pose_target.position.x)
        rospy.logerr(self.pose_target.position.y)
        rospy.logerr(self.pose_target.position.z)
        print(self.pose_target.position.x)
        print(self.pose_target.position.y)
        print(self.pose_target.position.z)
        rospy.sleep(2)

    def grasp(self):
        # Step2: Close Gripper joint value [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = -0.7
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan2 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def retreat(self):
        # Step3: Retreat/Move back [ARM GROUP]

        self.group_variable_values_arm_goal[0] = 0
        self.group_variable_values_arm_goal[1] = -1.7
        self.group_variable_values_arm_goal[2] = 1.43
        self.group_variable_values_arm_goal[3] = -1.76
        self.group_variable_values_arm_goal[4] = -1.51
        self.group_variable_values_arm_goal[5] = -1.6298
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan3 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)

    def rotate(self):
        # Step4: Turn Shoulder Joint 180 degrees [ARM GROUP]

        self.group_variable_values_arm_goal[0] = -3.14
        self.group_variable_values_arm_goal[1] = -1.0
        self.group_variable_values_arm_goal[2] = 1.43
        self.group_variable_values_arm_goal[3] = -1.76
        self.group_variable_values_arm_goal[4] = -1.51
        self.group_variable_values_arm_goal[5] = -1.6298
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan4 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)

    def release_object(self):
        # Step5: Open Gripper [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 0.2
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan5 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def main(self):
        if self.trigger is False:
            rospy.loginfo("Subscribing to /graspable_object_pose")
            pick_place_object.listener()

            pick_place_object.pregrasp()
            rospy.loginfo('Going to Goal Pose..')
            self.trigger = True

        if self.trigger is True:
            pick_place_object.grasp()
            rospy.loginfo('Retreating..')
            pick_place_object.retreat()
            rospy.loginfo('Rotating..')
            pick_place_object.rotate()
            rospy.loginfo('Releasing object..')
            pick_place_object.release_object()

            rospy.loginfo('Shuting Down ..')
            moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('pick_place_node_part_7', anonymous=True)
    perception_object = ObjectFilter()
    pick_place_object = Pick_Place_EE_Pose()
    try:
        perception_object.run()

        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
