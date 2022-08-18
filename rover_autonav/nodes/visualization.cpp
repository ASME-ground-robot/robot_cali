#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class Visualizer
{
     public:
	//The callback function for subscribing to the odometry message
	void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg_in);
	//The constructor of the class
	Visualizer(ros::NodeHandle nh);
     private:
	//Define the publisher and subscriber
	ros::Subscriber sub;
	ros::Publisher pub;
};

	//Define the constructor
Visualizer::Visualizer(ros::NodeHandle nh) {
	//Initialize publisher and subscriber
	sub=nh.subscribe("odom", 20, &Visualizer::callbackOdometry, this);
	pub=nh.advertise<visualization_msgs::Marker>("visual_position",20);
}

void Visualizer::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg_in)
{
	//Store the message
	nav_msgs::Odometry msg = *msg_in;

	//Generate visualization message
	visualization_msgs::Marker pose;
	pose.header.frame_id= "/map";
	pose.header.stamp = ros::Time::now();
	pose.ns = "rover_autonav";
	pose.action = visualization_msgs::Marker::ADD;
	pose.type = visualization_msgs::Marker::ARROW;
	pose.id=0;
	pose.scale.x=0.5;
	pose.scale.y=0.1;
	pose.scale.z=0.1;
	pose.color.r=1.0;
	pose.color.a=1.0;

	pose.pose.position.x = msg.pose.pose.position.x;
	pose.pose.position.y = msg.pose.pose.position.y;
	pose.pose.position.z = msg.pose.pose.position.z;

	pose.pose.orientation.x = msg.pose.pose.orientation.x;
	pose.pose.orientation.y = msg.pose.pose.orientation.y;
	pose.pose.orientation.z = msg.pose.pose.orientation.z;
	pose.pose.orientation.w = msg.pose.pose.orientation.w;

	//publish the visualization message
	pub.publish(pose);
}


int main(int argc, char **argv)
{
	//Initialize the ROS System
	ros::init(argc, argv, "rover_vis");
	//Initialize node handle
	ros::NodeHandle nh;
	//Create an instance of the class
	Visualizer visualizer(nh);
	//Start the loop
	ros::spin();
	return 0;
}
