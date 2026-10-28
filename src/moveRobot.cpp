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


// --- Global Variables ---
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
} param;

struct robotController
{
	double Xr;
	double Yr;
	double Thetar;
	double linear_vel;
	double angular_vel;
	double Xgoal;
	double Ygoal;
	sensor_msgs::LaserScan scan;
	
	bool isGoal = false;
	bool isOdom = false;
	bool isScan = false;

} robot;

enum State { FACE_GOAL, GO, ROTATE, FORWARD, DONE };
State state = FACE_GOAL;

// publisher of commands
ros::Publisher cmd_vel_pub;
float codeVelMax;
ros::Time start;


// --- Callback functions ---
//Callback of the topic myGoals
void Point_callback(const geometry_msgs::Point::ConstPtr& msg)
{
	robot.Xgoal = msg->x;
	robot.Ygoal = msg->y;
	robot.isGoal = true;
	ROS_INFO("Recieved  x:[%f], y:[%f]", robot.Xgoal, robot.Ygoal);
}


void callbackOdom(const nav_msgs::Odometry odom) {
	robot.Xr = odom.pose.pose.position.x;
	robot.Yr = odom.pose.pose.position.y;
	robot.Thetar = tf::getYaw(odom.pose.pose.orientation);
	robot.isOdom = true;
}

void callbackLaser(const sensor_msgs::LaserScan& most_intense) {
	robot.scan = most_intense;
	robot.isScan = true;
}

bool isObstacle(){
	int length = robot.scan.ranges.size();
	for(int i = 0; i<length; i++){
		if(robot.scan.ranges[i] < param.CRIT_DIST) return true;
	}
	return false;
}


double rotVel(double ori_error){
	if(fabs(ori_error) > param.ORI_ERROR)
		return param.K_ROT_MAX * param.V_MAX_ROT * ori_error;
	else
		return param.K_ROT_MIN * param.V_MAX_ROT * ori_error;	
}
	
void simpleAvoidanceController(){
	// common calculations
	double dx = robot.Xgoal - robot.Xr;
	double dy = robot.Ygoal - robot.Yr;
	double distance = sqrt(pow(dx, 2) + pow(dy, 2));
	double thetaGoal = atan2(dy, dx);
	double ori_error = atan2(sin(thetaGoal - robot.Thetar),
                         cos(thetaGoal - robot.Thetar));	
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	
	switch(state) {
	case FACE_GOAL:
		cmd_vel.angular.z = rotVel(ori_error);
		if(fabs(ori_error) < param.ORI_ERROR) state = GO;
		ROS_INFO("[FACE_GOAL] distance=%lf, angle=%lf\n", distance, ori_error * 180 / M_PI);
		break;
	case GO:
		cmd_vel.angular.z = rotVel(ori_error);
		cmd_vel.linear.x = param.V_MAX_DES;
		if(isObstacle()) state = ROTATE;
		if(distance <= param.D_OBJ) state = DONE;
		ROS_INFO("[GO] distance=%lf, angle=%lf\n", distance, ori_error * 180 / M_PI);
		break;
	case ROTATE:
		cmd_vel.angular.z = param.V_MAX_ROT;
		cmd_vel.linear.x = 0.0;
		ROS_INFO("[ROTATE] distance=%lf, angle=%lf\n", distance, ori_error * 180 / M_PI);
		if(!isObstacle()) {
			state = FORWARD;
			start = ros::Time::now(); // due to clock rate it will trigger 200 not 150
		}
		break;
	case FORWARD:
		cmd_vel.linear.x = param.V_MAX_DES;
		if(ros::Time::now() - start > ros::Duration(param.T_AVOID_OBS)) state = FACE_GOAL;
		ROS_INFO("[FORWARD] distance=%lf, angle=%lf\n", distance, ori_error * 180 / M_PI);
		break;
	case DONE:
		ROS_INFO("[DONE] distance=%lf, angle=%lf\n", distance, ori_error * 180 / M_PI);
		break;		
	}
	
	cmd_vel_pub.publish(cmd_vel);	
}


// --- main ---
int main(int argc, char **argv)
{
	
	ros::init(argc, argv,"moveRobot");
	ros::Time::init();
	
	// Read parameters and Print as Info
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
	ros::NodeHandle nh;
	//Build a string with topic name
	std::ostringstream parent;
	parent << "robot_" << param.ID_ROBOT;
	
	// odom subscriber
    string odom_topic_name = parent.str();
    odom_topic_name += "/odom";
	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callbackOdom);
	
	// laser subscriber
	string sonar_scan_topic_name = parent.str();
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber laser_sub = nh.subscribe(sonar_scan_topic_name, 1, callbackLaser);
	
	// goal subscriber
	// string goal_topic_name = parent.str();
	string goal_topic_name = "/myGoals";
	ros::Subscriber goal_sub = nh.subscribe(goal_topic_name, 10, Point_callback);
	
	// velocity topic publisher
	string cmd_vel_topic_name = parent.str();
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
    
	ros::Rate rate(10);           // 10 Hz control loop
	while (ros::ok()) {
		ros::spinOnce();          // run any waiting callbacks, then return
		// wait for goal and odom
		if (robot.isGoal && robot.isOdom && robot.isScan) simpleAvoidanceController();
		rate.sleep();
	}
	
	return 0;

	
	
	
}

