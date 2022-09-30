#! /usr/bin/env python
import rospy
import actionlib
import os
import rosparam
from rover_autonav.srv import GoToPoi, GoToPoiResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback


class GoToPOI(object):
    def __init__(self):

        self.srv_server = rospy.Service(
            '/go_to_point', GoToPoi, self.main_callback)
        self.client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.client.wait_for_server()

        # Load the parameters in test_params.yaml into the ROS Parameter Server
        os.chdir("/home/jason/catkin_ws/src/rover_autonav/params")
        self.paramlist = rosparam.load_file(
            "goal_poses.yaml", default_namespace=None, verbose=False)
        for params, ns in self.paramlist:
            rosparam.upload_params(ns, params)

        # Get the parameters from the ROS Param Server
        self.position_x = rosparam.get_param('grasp_position/position/x')
        self.position_y = rosparam.get_param('grasp_position/position/y')
        self.position_z = rosparam.get_param('grasp_position/position/z')
        self.orientation_x = rosparam.get_param('grasp_position/orientation/x')
        self.orientation_y = rosparam.get_param('grasp_position/orientation/y')
        self.orientation_z = rosparam.get_param('grasp_position/orientation/z')
        self.orientation_w = rosparam.get_param('grasp_position/orientation/w')

        self.position2_x = rosparam.get_param('release_position/position/x')
        self.position2_y = rosparam.get_param('release_position/position/y')
        self.position2_z = rosparam.get_param('release_position/position/z')
        self.orientation2_x = rosparam.get_param(
            'release_position/orientation/x')
        self.orientation2_y = rosparam.get_param(
            'release_position/orientation/y')
        self.orientation2_z = rosparam.get_param(
            'release_position/orientation/z')
        self.orientation2_w = rosparam.get_param(
            'release_position/orientation/w')

        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
        # # waits until the action server is up and running
        # self.client.wait_for_server()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

################################################################

    def feedback_callback(self, feedback):
        print('[Feedback] Going to Point of Interest!!!')

    def main_callback(self, request):

        goal = MoveBaseGoal()
        if request.label == 'grasp_position':
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = self.position_x
            goal.target_pose.pose.position.y = self.position_y
            goal.target_pose.pose.position.z = self.position_z
            goal.target_pose.pose.orientation.x = self.orientation_x
            goal.target_pose.pose.orientation.y = self.orientation_y
            goal.target_pose.pose.orientation.z = self.orientation_z
            goal.target_pose.pose.orientation.w = self.orientation_w

        elif request.label == 'release_position':
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = self.position2_x
            goal.target_pose.pose.position.y = self.position2_y
            goal.target_pose.pose.position.z = self.position2_z
            goal.target_pose.pose.orientation.x = self.orientation2_x
            goal.target_pose.pose.orientation.y = self.orientation2_y
            goal.target_pose.pose.orientation.z = self.orientation2_z
            goal.target_pose.pose.orientation.w = self.orientation2_w

        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        self.client.wait_for_result()

        print('[Result] State: %d' % (self.client.get_state()))
######################################################################

        response = GoToPoiResponse()
        response.success = 'OK, Service Finished correctly'
        return response


if __name__ == '__main__':
    rospy.init_node('go_to_poi', anonymous=True)
    rb1_object = GoToPOI()
    rospy.spin()
