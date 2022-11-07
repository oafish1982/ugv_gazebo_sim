#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("lite6");
    moveit::planning_interface::MoveGroupInterface gripper("xarm_gripper");

    gripper.setNamedTarget("close");
    gripper.move();

    std::string end_effector_link = arm.getEndEffectorLink(); //获取终端link的名称
    std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;//打印终端名称
   
    std::string reference_frame = "world"; //设置目标位置所使用的参考坐标系，参数为urdf文件中设置的基础连杆坐标系
    arm.setPoseReferenceFrame(reference_frame);
    arm.allowReplanning(true); //当运动规划失败后，允许重新规划
    arm.setGoalJointTolerance(0.001);//设置关节允许误差
    arm.setGoalPositionTolerance(0.001); //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalOrientationTolerance(0.01);   
    arm.setMaxAccelerationScalingFactor(0.2); //设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;//打印目前机器人的位姿
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z
       <<","<<now_pose.orientation.w<<"]"<<std::endl;
  
    arm.setNamedTarget("home");// 控制机械臂先回到初始化位置
    arm.move();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;//声明一个vector容器保存路径点
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.19091;
    pose1.position.y = 0.267263;	
    pose1.position.z = 0.352276;
    pose1.orientation.x = -0.924232;
    pose1.orientation.y = 0.00066903;
    pose1.orientation.z = 0.00101212;
    pose1.orientation.w = 0.38183;
    waypoints.push_back(pose1);
// 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.002;
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
// 笛卡尔空间下的路径规划函数，包含四个参数，并返回一个fraction值确定是否规划成功
        attempts++;        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
	// 生成机械臂的运动规划数据
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;
	// 执行运动
	arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    
    ros::shutdown(); 
    return 0;
}

