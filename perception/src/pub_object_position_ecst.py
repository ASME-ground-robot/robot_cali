#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose



class SurfaceObjectFilter:

    def __init__(self, table_height_init=0.63, error_height=0.05):

        self._rate = rospy.Rate(5)

        self.table_height = table_height_init
        self._error_height = error_height
        self.object_dict = {}
        self.surface_topic = "/surface_objects"
        self._check_surface_ready()

        rospy.Subscriber(self.surface_topic, Marker, self.surface_callback)
        self.my_pub = rospy.Publisher(
            '/graspable_object_pose', Pose, queue_size=10)
        self.graspable_object_position = Pose()

        rospy.loginfo('Ready to detect Surfaces!')

    def _check_surface_ready(self):
        self._surface_data = None
        while self._surface_data is None and not rospy.is_shutdown():
            try:
                self._surface_data = rospy.wait_for_message(
                    self.surface_topic, Marker, timeout=1.0)
                rospy.logdebug("Current "+self.surface_topic +
                               " READY=>" + str(self._surface_data))

            except:
                rospy.logerr("Current "+self.surface_topic +
                             " not ready yet, retrying.")

    def update_table_height(self, new_table_height):
        self.table_height = new_table_height

    def look_for_table_surface(self, z_value):

        delta_min = z_value - self._error_height
        delta_max = z_value + self._error_height
        is_the_table = delta_min < self.table_height < delta_max

        return is_the_table

    def surface_callback(self, msg):

        name = msg.ns
        self.graspable_object_position = msg.pose
        # object_pose = msg.pose

        if "surface_" in name and "_axes" in name:
            # We check the heigh in z to see if its the table
            if self.look_for_table_surface(msg.pose.position.z):
                if name in self.object_dict:
                    rospy.loginfo("This object was already found")
                else:
                    # add item in dict: {'name_object': graspable_object_position}
                    self.object_dict[name] = self.graspable_object_position
                    rospy.loginfo("Found New Surface=")
        else:
            rospy.logdebug("Surface Object Not found "+str(name))

    def get_object_dict_detected(self):
        return self.object_dict

    def run(self):

        while not rospy.is_shutdown():
            objects_detected = self.get_object_dict_detected()
            rospy.loginfo(str(objects_detected))
            self.my_pub.publish(self.graspable_object_position)
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_data_extract_node', log_level=rospy.INFO)

    try:
        SurfaceObjectFilter().run()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
