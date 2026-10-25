//roscpp Core header 
#include "ros/ros.h"
#include "geometry_msgs/Point.h" 

int main(int argc, char **argv)

{
	ros::init(argc, argv,"talkerGoals");
	ros::NodeHandle node_obj;
	ros::Publisher point_publisher = node_obj.advertise<geometry_msgs::Point>("myGoals",10); // name topic, queue length
	// maybe investigate 3rd param "latch" for advertise
	
	// Variables
	double X_GOAL = 0.0; // x coordinate meters
	double Y_GOAL = 0.0; // y coordinate meters
	double T_GOAL = 3; // time interval between seconds (rec 3 - 7)
	int number_count = 0;
	
	// Try to get parameters
	ros::NodeHandle ns("~");
	if(! ns.getParam("X_GOAL", X_GOAL)) ROS_WARN("X_GOAL FAILED");
	if(! ns.getParam("Y_GOAL", Y_GOAL)) ROS_WARN("Y_GOAL FAILED");
	if(! ns.getParam("T_GOAL", T_GOAL)) ROS_WARN("T_GOAL FAILED");
	
	ros::Rate loop_rate(1 / T_GOAL);
	geometry_msgs::Point msg;
	msg.x = X_GOAL;
	msg.y = Y_GOAL;
	
	// Loop
	while (ros::ok())
	{
		
		ROS_INFO("%d: (%f, %f)", number_count, msg.x, msg.y);
		point_publisher.publish(msg);
		// ros::spinOnce(); not needed since no callbacks
		loop_rate.sleep();
		number_count++;
	}
	return 0;
}

