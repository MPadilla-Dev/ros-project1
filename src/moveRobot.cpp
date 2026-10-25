#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "assert.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"


#include <string>
#include <sstream>
#include <iostream>

using namespace std;
using std::ostringstream;

// structure for parameters
struct parameters 
{
	int ID_ROBOT;
	int ALGOR;
	double CRIT_DIST;
	double D_OBJ;
	double V_MAX_DES;
	double V_MAX_ROT;
	double K_ROT_MIN;
	double K_ROT_MAX;
	double ORI_ERROR;
	double T_AVOID_OBS;
	double DIST_LEADER;
	int ROBOT_ROLE;
	int ID_LEADER;
	double W_1;
	double W_2;
	int T_WAIT;
};

//Callback of the topic myGoals
void Point_callback(const geometry_msgs::Point::ConstPtr& msg)
{
	ROS_INFO("Recieved  x:[%f], y:[%f]", msg->x, msg->y);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv,"moveRobot");
	
	// Read parameters and Print as Info
	parameters param;
	ros::NodeHandle nh_priv("~");
	nh_priv.param<int>("ID_ROBOT", param.ID_ROBOT, 0);
	nh_priv.param<int>("ALGOR", param.ALGOR, 1);
	nh_priv.param<double>("CRIT_DIST", param.CRIT_DIST, 1.2);
	nh_priv.param<double>("D_OBJ", param.D_OBJ, 0.6);
	nh_priv.param<double>("V_MAX_DES", param.V_MAX_DES, 1.2);
	nh_priv.param<double>("V_MAX_ROT", param.V_MAX_ROT, 1.1);
	nh_priv.param<double>("K_ROT_MIN", param.K_ROT_MIN, 0);
	nh_priv.param<double>("K_ROT_MAX", param.K_ROT_MAX, 0);
	nh_priv.param<double>("ORI_ERROR", param.ORI_ERROR, 0);
	nh_priv.param<double>("T_AVOID_OBS", param.T_AVOID_OBS, 0);
	nh_priv.param<double>("DIST_LEADER", param.DIST_LEADER, 0);
	nh_priv.param<int>("ROBOT_ROLE", param.ROBOT_ROLE, 0);
	nh_priv.param<int>("ID_LEADER", param.ID_LEADER, 0);
	nh_priv.param<double>("W_1", param.W_1, 0);
	nh_priv.param<double>("W_2", param.W_2, 0);
	nh_priv.param<int>("T_WAIT", param.T_WAIT, 225);

	ROS_INFO("ID_ROBOT=%d", param.ID_ROBOT);
	ROS_INFO("ALGOR=%d", param.ALGOR);
	ROS_INFO("CRIT_DIST=%lf", param.CRIT_DIST);
	ROS_INFO("D_OBJ=%lf", param.D_OBJ);
	ROS_INFO("V_MAX_DES=%lf", param.V_MAX_DES);
	ROS_INFO("V_MAX_ROT=%lf", param.V_MAX_ROT);
	ROS_INFO("K_ROT_MIN=%lf", param.K_ROT_MIN);
	ROS_INFO("K_ROT_MAX=%lf", param.K_ROT_MAX);
	ROS_INFO("ORI_ERROR=%lf", param.ORI_ERROR);
	ROS_INFO("T_AVOID_OBS=%lf", param.T_AVOID_OBS);
	ROS_INFO("DIST_LEADER=%lf", param.DIST_LEADER);
	ROS_INFO("ROBOT_ROLE=%d", param.ROBOT_ROLE);
	ROS_INFO("ID_LEADER=%d", param.ID_LEADER);
	ROS_INFO("W_1=%lf", param.W_1);
	ROS_INFO("W_2=%lf", param.W_2);
	ROS_INFO("T_WAIT=%d", param.T_WAIT);


	
	
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber sub = node_obj.subscribe("/myGoals", 10, Point_callback);
	//Spinning the node
	ros::spin();
	return 0;
}

