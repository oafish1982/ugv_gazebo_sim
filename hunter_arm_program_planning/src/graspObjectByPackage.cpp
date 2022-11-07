#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class graspUsePackage
{
  public:
    graspUsePackage(ros::NodeHandle n_, geometry_msgs::Pose home_pose_, geometry_msgs::Pose place_pose_):nh_(n_), arm("lite6"), gripper("xarm_gripper")
    {
      home_pose = home_pose_;
      target_pose = home_pose_; 
      place_pose = place_pose_;
      arm.allowReplanning(true); 
      arm.setGoalPositionTolerance(0.05); 
      arm.setGoalOrientationTolerance(0.02);
    }
    void goHome()
    {
      arm.setPoseTarget(home_pose);
      arm.move();
      target_pose.orientation = (arm.getCurrentPose()).pose.orientation;
    }
    void goGrasp()
    {
      try
      {
        this->tf_object_to_base.waitForTransform("/link_base", "/object_26", ros::Time(0), ros::Duration(50.0));
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
        ros::Duration(1.0).sleep();
      }
      try
      {
        this->tf_object_to_base.lookupTransform("/link_base", "/object_26", ros::Time(0), (this->object_to_base));
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
      }
      // ROS_INFO("getted transform");
      // std::cout<< " X-Co-ordinate in Robot Frame :" << object_to_base.getOrigin().getX() << std::endl;
      // std::cout<< " Y-Co-ordinate in Robot Frame :" << object_to_base.getOrigin().getY() << std::endl;
      // std::cout<< " Z-Co-ordinate in Robot Frame :" << object_to_base.getOrigin().getZ() << std::endl;    
      target_pose.position.x = object_to_base.getOrigin().getX() ;
      target_pose.position.y = object_to_base.getOrigin().getY() ;
      target_pose.position.z = object_to_base.getOrigin().getZ() + 0.04;
      arm.setPoseTarget(target_pose);
      arm.move();
      ros::WallDuration(1.0).sleep();
      // target_pose.position.z = target_pose.position.z - 0.05;
      // arm.setPoseTarget(target_pose);
      // arm.move();

      // gripper.setNamedTarget("close");
      // gripper.move();      
    }
    void goThrow()
    {
      target_pose.position.z = target_pose.position.z + 0.08;
      arm.setPoseTarget(target_pose);
      arm.move();

      arm.setPoseTarget(place_pose);
      arm.move();
      //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      //bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      //arm.execute(my_plan);

      gripper.setNamedTarget("open");
      gripper.move();
    }
    void graspLoop()
    {
      while(true)
      {
        goHome();
        ros::WallDuration(2.0).sleep();
        goGrasp();
        goThrow();
      }
    }
  private:
    ros::NodeHandle nh_;
	  moveit::planning_interface::MoveGroupInterface arm;
	  moveit::planning_interface::MoveGroupInterface gripper;
    
  	tf::StampedTransform object_to_base;
  	tf::TransformListener tf_object_to_base;
    
    geometry_msgs::Pose home_pose;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose place_pose;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graspWithPackage");
  ros::NodeHandle n;

  geometry_msgs::Pose home_pose;
  home_pose.position.x = 0.207254;
  home_pose.position.y = 8.81722e-05;
  home_pose.position.z = 0.201665;
  home_pose.orientation.x = 0.999963;
  home_pose.orientation.y = 0.000934649;
  home_pose.orientation.z = 0.0085096;
  home_pose.orientation.w = 8.94135e-05;

  geometry_msgs::Pose place_pose;
  place_pose.position.x = -0.132207;
  place_pose.position.y = -0.334207;
  place_pose.position.z = 0.0460801;
  place_pose.orientation.x = 0.709168;
  place_pose.orientation.y = -0.704144;
  place_pose.orientation.z = 0.0250833;
  place_pose.orientation.w = 0.0251638;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  graspUsePackage simGrasp(n, home_pose, place_pose);
  // simGrasp.graspLoop();
  simGrasp.goHome();
  // simGrasp.goGrasp();
  // simGrasp.goThrow();  

  return 0;
}


