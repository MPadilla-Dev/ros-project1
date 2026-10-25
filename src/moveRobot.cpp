#include "ros/ros.h"
#include "geometry_msgs/Point.h"

//Callback of the topic myGoals
void Point_callback(const geometry_msgs::Point::ConstPtr& msg)
{
	ROS_INFO("Recieved  x:[%f], y:[%f]", msg->x, msg->y);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv,"moveRobot");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber sub = node_obj.subscribe("/myGoals", 10, Point_callback);
	//Spinning the node
	ros::spin();
	return 0;
}

