#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "demo"); //初始化
    ros::AsyncSpinner spinner(1); //多线程
    spinner.start(); //开启新的线程
    moveit::planning_interface::MoveGroupInterface arm("lite6");
    geometry_msgs::PoseStamped now_pose = arm.getCurrentPose();//打印目前机器人的位姿
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.pose.position.x<<","<<now_pose.pose.position.y<<","<<now_pose.pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.pose.orientation.x<<","<<now_pose.pose.orientation.y<<","<<now_pose.pose.orientation.z
       <<","<<now_pose.pose.orientation.w<<"]"<<std::endl;

    Eigen::Quaterniond quaternion(now_pose.pose.orientation.w, now_pose.pose.position.x, now_pose.pose.position.y, now_pose.pose.position.z);
    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0,1,2);
    std::cout<<"欧拉角为:"<<eulerAngle[0]/3.1415926*180<<","<<eulerAngle[1]/3.1415926*180<<","<<eulerAngle[2]/3.1415926*180<<std::endl;

    const robot_state::JointModelGroup* joint_model_group = arm.getCurrentState()->getJointModelGroup("lite6");
    moveit::core::RobotStatePtr current_state = arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);//获取关节空间位姿
    //joint_group_positions[0] = 1.5707963; 
    //joint_group_positions[1] = -1.9198621;
    //joint_group_positions[2] = 1.5707963;
    //joint_group_positions[3] = -1.5707963;
    //joint_group_positions[4] = -1.5707963;
    //joint_group_positions[5] = -0;
    std::cout<<"now Robot orientation:"<<joint_group_positions[0]<<","<<joint_group_positions[1]<<","<<joint_group_positions[2]<<","<<joint_group_positions[3]<<","<<joint_group_positions[4]<<","<<joint_group_positions[5]<<","<<std::endl;
    //arm.setJointValueTarget(joint_group_positions);
    //arm.move();
    ros::shutdown(); 
    return 0;
}

