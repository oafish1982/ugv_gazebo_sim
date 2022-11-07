
#ifndef PROBOT_GRASPING_DEMO
#define PROBOT_GRASPING_DEMO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "std_msgs/String.h"
#include <vector>
#include "hunter_arm_program_planning/visionManager.h"

class GraspingDemo
{
  private:

	ros::NodeHandle nh_;

	moveit::planning_interface::MoveGroupInterface arm;
	moveit::planning_interface::MoveGroupInterface gripper;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	cv_bridge::CvImagePtr cv_ptr;
	VisionManager vMng_;

	tf::StampedTransform camera_to_robot_;
	tf::TransformListener tf_camera_to_robot;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::MoveItErrorCode success;
	geometry_msgs::PoseStamped homePose;
	float pregrasp_x, pregrasp_y, pregrasp_z;
	bool grasp_running;

	void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale);
	void attainPosition(float x, float y, float z);
	void attainObject();
	void grasp();
	void lift();

  public:

	tf::Vector3 obj_camera_frame, obj_robot_frame;

	GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length = 1, float breadth = 0.6);
	void imageCb(const sensor_msgs::ImageConstPtr &msg);
	void initiateGrasping();
	void goHome();
	void add_barrier();
};

#endif
