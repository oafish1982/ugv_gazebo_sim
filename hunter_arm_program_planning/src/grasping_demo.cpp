#include "hunter_arm_program_planning/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) :
    it_(n_), 
    arm("lite6"), 
    gripper("xarm_gripper"), 
    vMng_(length, breadth)
{
  this->nh_ = n_;
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;
  arm.setPoseReferenceFrame("link_base");
  arm.setMaxAccelerationScalingFactor(1);
  arm.setMaxVelocityScalingFactor(1);
  gripper.setMaxAccelerationScalingFactor(1);
  gripper.setMaxVelocityScalingFactor(1);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();

  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
  homePose = arm.getCurrentPose();
  ros::WallDuration(1.0).sleep();
  grasp_running = false;
  image_sub_ = it_.subscribe("/d435/color/image_raw", 1, &GraspingDemo::imageCb, this);
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    try
    {
      this->tf_camera_to_robot.waitForTransform("/link_base", "/d435_color_optical_frame", ros::Time(0), ros::Duration(50.0));
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
      ros::Duration(1.0).sleep();
    }
    try
    {
      this->tf_camera_to_robot.lookupTransform("/link_base", "/d435_color_optical_frame", ros::Time(0), (this->camera_to_robot_));
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
    }

    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(0.43);
    obj_camera_frame.setY(obj_y);
    obj_camera_frame.setX(obj_x);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
  ROS_INFO("The attain position function called");	
  // std::cout<<"now Robot position: [x,y,z]: ["<<currPose.pose.position.x<<","<<currPose.pose.position.y<<","<<currPose.pose.position.z<<"]"<<std::endl;
  // std::cout<<"now Robot orientation: [x,y,z,w]: ["<<currPose.pose.orientation.x<<","<<currPose.pose.orientation.y<<","<<currPose.pose.orientation.z<<","<<currPose.pose.orientation.w<<"]"<<std::endl;

  geometry_msgs::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = 1;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  arm.setPoseTarget(target_pose);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }
  ros::WallDuration(1.0).sleep();
}

void GraspingDemo::attainObject()
{
  ROS_INFO("The attain Object function called");
  geometry_msgs::PoseStamped currPose = arm.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  // target_pose1.orientation.x = -3.75432e-05;
  // target_pose1.orientation.y = 0.00804158;
  // target_pose1.orientation.z = 0.000135543;
  // target_pose1.orientation.w = 0.999968;
  target_pose1.position.x = obj_robot_frame.getX();
  target_pose1.position.y = obj_robot_frame.getY();
  target_pose1.position.z = obj_robot_frame.getZ() + 0.08;
  arm.setPoseTarget(target_pose1);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }
  ros::WallDuration(1.0).sleep();

  target_pose1.position.z = target_pose1.position.z - 0.08;
  arm.setPoseTarget(target_pose1);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }
  ros::WallDuration(0.5).sleep();
}

void GraspingDemo::grasp()
{
  ROS_INFO("The Grasping function called");
  gripper.setNamedTarget("close");
  success = gripper.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 3);
  if(success)
  {
    gripper.execute(my_plan);
  }
  ros::WallDuration(0.8).sleep();  
}

void GraspingDemo::lift()
{
  ROS_INFO("The lift function called");
  ros::WallDuration(1.0).sleep();

  geometry_msgs::PoseStamped currPose = arm.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;
  target_pose1.position.z = target_pose1.position.z + 0.15;
  arm.setPoseTarget(target_pose1);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }

  target_pose1.position.x = -0.232641;
  target_pose1.position.y = -0.0468004;
  target_pose1.position.z = 0.2783;
  target_pose1.orientation.x = 0.0184491;
  target_pose1.orientation.y = -0.996985;
  target_pose1.orientation.z = 0.00801343;
  target_pose1.orientation.w = 0.0749355;
  arm.setPoseTarget(target_pose1);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }  

  ros::WallDuration(0.5).sleep();
  gripper.setNamedTarget("open");
    success = gripper.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 8);
  if(success)
  {
    gripper.execute(my_plan);
  }
}

void GraspingDemo::goHome()
{
  ROS_INFO("The gohome function called");
  arm.setPoseTarget(homePose);
  success = arm.plan(my_plan);  
  ROS_INFO("Plan (pose goal) %s", success? "":"FAILED");
  scale_trajectory_speed(my_plan, 4);
  if(success)
  {
    arm.execute(my_plan);
  }
}

void GraspingDemo::initiateGrasping()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();

  arm.allowReplanning(true); //当运动规划失败后，允许重新规划
  //arm.setGoalJointTolerance(0.1);//设置关节允许误差
  arm.setGoalPositionTolerance(0.005); //设置位置(单位：米)的允许误差
  // arm.setGoalOrientationTolerance(0.02);//设置姿态误差（单位：弧度）的允许误差
  if(grasp_running==true)
  {
    attainObject();
    grasp();
    lift();
    goHome();    
  }

  grasp_running = false;
}

void GraspingDemo::scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale)
{
    int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();//获取关节个数
    
    for(int i=0; i<plan.trajectory_.joint_trajectory.points.size(); i++)//通过for循环对plan中所有的轨迹点作一个遍历
    {
        plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/scale;//速度变慢，时间变长为 1/scale 倍
        
        for(int j=0; j<n_joints; j++)//遍历各个关节，每一个关节的速度和加速度数据都要作一个尺度的变化
        {
            plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;//速度变化为原来的 scale
            plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale*scale;//加速度变化为原来的 scale * scale
        }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;


  length = 0.1;
  breadth = 0.1;  
  pregrasp_x = 0.27115;
  pregrasp_y =9.27577e-06;
  pregrasp_z = 0.306812;

  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
  ros::WallDuration(1.0).sleep();
  while (ros::ok())
  {
    ros::spinOnce();
    simGrasp.initiateGrasping();
  }

  return 0;
}
