#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rviz_visual_tools/rviz_visual_tools.h>


#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass{
  
public:
  explicit MyCompetitionClass(ros::NodeHandle & node) : current_score_(0), has_been_zeroed_(false){
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
  }


  /// Called when a new JointState message is received.
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states_ = *joint_state_msg;
    if (!has_been_zeroed_) {
      has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      //send_arm_to_zero_state();
    }
  }



  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state() {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("iiwa_joint_1");
    msg.joint_names.push_back("iiwa_joint_2");
    msg.joint_names.push_back("iiwa_joint_3");
    msg.joint_names.push_back("iiwa_joint_4");
    msg.joint_names.push_back("iiwa_joint_5");
    msg.joint_names.push_back("iiwa_joint_6");
    msg.joint_names.push_back("iiwa_joint_7");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.001);
    //ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
  }

  ////////// SENSORS ///////////////

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Logical camera: '" << image_msg->models.size() << "' objects.");

    for(int i=0; i<image_msg->models.size(); ++i){
      ROS_INFO_STREAM_THROTTLE(10, "Object detected is a " << image_msg->models[i].type);
    }
  }

  /// Called when a new Proximity message is received.
  void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    }
  }

//  void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
//    size_t number_of_valid_ranges = std::count_if(msg->ranges.begin(), msg->ranges.end(), std::isfinite<float>);
//    if (number_of_valid_ranges > 0) {
//      ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
//    }
//  }

//  void depth_camera_callback(const sensor_msgs::DepthCameraScan::ConstPtr & msg){

//  }

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher joint_trajectory_publisher_;
  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState current_joint_states_;
  bool has_been_zeroed_;
};

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

/*
void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}
*/
void first_task(){

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // initialize rviz and clear all markers
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  // load rviz control
  visual_tools.loadRemoteControl();

  // set markers
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // update rviz frame
  visual_tools.trigger();

  ROS_INFO("Reference frame: ");
  ROS_INFO(move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: ");
  ROS_INFO(move_group.getEndEffectorLink().c_str());

  // set target pose
  geometry_msgs::Pose target_pose1;
  // grab gasket part top-left
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = -0.550948;
  //target_pose1.position.x = -0.900838;
  target_pose1.position.y = 1.072163;
  //target_pose1.position.y = 2.623346;
  target_pose1.position.z = 1.467166;
  //target_pose1.position.z = 0.785;
  move_group.setPoseTarget(target_pose1);
  ROS_INFO("Target pose set");

  // path planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal)");
  if(success)
    ROS_INFO("PASSED");
  else
    ROS_INFO("FAILED");

  // visualize planned path
  ROS_INFO("Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // move to target
  move_group.move();
}


int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "vs_qual_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe("/ariac/current_score", 10, &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe("/ariac/competition_state", 10, &MyCompetitionClass::competition_state_callback, &comp_class);


  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe("/ariac/orders", 10, &MyCompetitionClass::order_callback, &comp_class);


  // Subscribe to the '/ariac/joint_states' topic.
  ros::Subscriber joint_state_subscriber = node.subscribe("/ariac/joint_states", 10, &MyCompetitionClass::joint_state_callback, &comp_class);


  // Subscribe to the '/ariac/proximity_sensor_1' topic.
  //ros::Subscriber proximity_sensor_subscriber = node.subscribe("/ariac/proximity_sensor_1", 10, proximity_sensor_callback);


  // Subscribe to the '/ariac/break_beam_1_change' topic.
  ros::Subscriber break_beam_subscriber = node.subscribe("/ariac/break_beam_1_change", 10, &MyCompetitionClass::break_beam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe("/ariac/logical_camera_1", 10, &MyCompetitionClass::logical_camera_callback, &comp_class);

  // Subscribe to the '/ariac/laser_profiler_1' topic.
  //ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler_1", 10, &MyCompetitionClass::laser_profiler_callback, &comp_class);

  //ros::Subscriber depth_camera_subscriber = node.subscribe("/ariac/depth_camera_1", 10, &MyCompetitionClass::depth_camera_callback);

  ROS_INFO("Setup complete.");
  
  start_competition(node);

  first_task();
  
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
