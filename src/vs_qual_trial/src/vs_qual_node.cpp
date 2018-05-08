#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf/transform_datatypes.h>

#include <geometric_shapes/shape_operations.h>

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
    
    init_subscribers(node);
    init_publishers(node);

    start_competition(node);
    first_task();
  }

  void init_subscribers(ros::NodeHandle & node){

    // Subscribe to the '/ariac/current_score' topic.
    current_score_subscriber = node.subscribe("/ariac/current_score", 10, &MyCompetitionClass::current_score_callback, this);

    // Subscribe to the '/ariac/competition_state' topic.
    competition_state_subscriber = node.subscribe("/ariac/competition_state", 10, &MyCompetitionClass::competition_state_callback, this);

    // Subscribe to the '/ariac/orders' topic.
    orders_subscriber = node.subscribe("/ariac/orders", 10, &MyCompetitionClass::order_callback, this);

    // Subscribe to the '/ariac/joint_states' topic.
    joint_state_subscriber = node.subscribe("/joint_states", 10, &MyCompetitionClass::joint_state_callback, this);

    // Subscribe to the '/ariac/proximity_sensor_1' topic.
    //ros::Subscriber proximity_sensor_subscriber = node.subscribe("/ariac/proximity_sensor_1", 10, proximity_sensor_callback);

    // Subscribe to the '/ariac/break_beam_1_change' topic.
    break_beam_subscriber = node.subscribe("/ariac/break_beam_1_change", 10, &MyCompetitionClass::break_beam_callback, this);

    // Subscribe to the '/ariac/logical_camera_1' topic.
    logical_camera_subscriber = node.subscribe("/ariac/logical_camera_1", 10, &MyCompetitionClass::logical_camera_callback, this);

    // Subscribe to the '/ariac/laser_profiler_1' topic.
    //ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler_1", 10, &MyCompetitionClass::laser_profiler_callback, this);

    //ros::Subscriber depth_camera_subscriber = node.subscribe("/ariac/depth_camera_1", 10, &MyCompetitionClass::depth_camera_callback);
  }

  void init_publishers(ros::NodeHandle & node){

    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
    collision_object_pub = node.advertise<moveit_msgs::CollisionObject>( "collision_object", 1000 );
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
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg){
    
    ROS_INFO_STREAM_THROTTLE(10, "Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states_ = *joint_state_msg;
    if (!has_been_zeroed_) {
      has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state();
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
    ros::Duration(5.0).sleep();   // wait for rviz to update


  }

  ////////// SENSORS ///////////////

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10, "Logical camera: '" << image_msg->models.size() << "' objects.");

    piston_part_pose.resize(image_msg->models.size());

    for(int i=0; i<image_msg->models.size(); ++i){
      ROS_INFO_STREAM_THROTTLE(10, "Object detected is a " << image_msg->models[i].type);
      piston_part_pose.push_back(image_msg->models[i].pose);
      ros::Duration(0.2).sleep();
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

  /*
void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}
*/

  void loadObstacles(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface){

    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = move_group.getPlanningFrame();

    collision_object.id = "box1";

    // dimenions
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 6.0;
    primitive.dimensions[2] = 0.5;

    // pose
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.7;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.5;

    // add dimensions and pose to collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_object_pub.publish(collision_object);

    // if multiple objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    
    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("After addCollisionObjects");
  }

  void loadMeshObstacles(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface){

    Eigen::Vector3d container_scaling(1.1, 0.7, 0.8);   // scaling

    ///////////////////// Container ///////////////////////////////
    moveit_msgs::CollisionObject container_object;
    container_object.id = "container";
    std::string containerPath = "package://vs_qual_trial/models/shipping_container_short_ariac/meshes/ShippingContainer_Textured.obj";
    shapes::Mesh* m = shapes::createMeshFromResource(containerPath, container_scaling);
    ROS_INFO("shipping_container mesh loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    container_object.meshes.resize(1);
    container_object.mesh_poses.resize(1);
    container_object.meshes[0] = mesh;
    container_object.header.frame_id = move_group.getPlanningFrame();
    container_object.mesh_poses[0].position.x = 0.275;
    container_object.mesh_poses[0].position.y = -1.0;
    container_object.mesh_poses[0].position.z = 0.0;

    container_object.meshes.push_back(mesh);
    container_object.mesh_poses.push_back(container_object.mesh_poses[0]);
    container_object.operation = container_object.ADD;
    collision_object_pub.publish(container_object);


    ///////////////////////// Belt /////////////////////////
    Eigen::Vector3d belt_scaling(0.9, 0.7, 1.0);   // scaling

    moveit_msgs::CollisionObject belt_object;
    belt_object.id = "belt";
    std::string beltPath = "package://vs_qual_trial/models/shipping_container_conveyor_ariac/meshes/ShippingContainer_Textured.obj";
    m = shapes::createMeshFromResource(beltPath, belt_scaling);
    ROS_INFO("belt mesh loaded");

    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    belt_object.meshes.resize(1);
    belt_object.mesh_poses.resize(1);
    belt_object.meshes[0] = mesh;
    belt_object.header.frame_id = move_group.getPlanningFrame();
    belt_object.mesh_poses[0].position.x = 0.04;
    belt_object.mesh_poses[0].position.y = -1.0;
    belt_object.mesh_poses[0].position.z = 0.0;

    belt_object.meshes.push_back(mesh);
    belt_object.mesh_poses.push_back(belt_object.mesh_poses[0]);
    belt_object.operation = belt_object.ADD;
    collision_object_pub.publish(belt_object);

    ////////////// Yellow Bin 1 ////////////////////
    Eigen::Vector3d bin_scaling(1.0, 2.1, 1.0);   // scaling

    moveit_msgs::CollisionObject bin1_object;
    bin1_object.id = "bin1";
    std::string bin1Path = "package://vs_qual_trial/models/yellow_bin_ariac/meshes/yellow_bin.obj";
    m = shapes::createMeshFromResource(bin1Path, bin_scaling);
    ROS_INFO("bin1 mesh loaded");

    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    bin1_object.meshes.resize(1);
    bin1_object.mesh_poses.resize(1);
    bin1_object.meshes[0] = mesh;
    bin1_object.header.frame_id = move_group.getPlanningFrame();
    bin1_object.mesh_poses[0].position.x = -0.775;
    bin1_object.mesh_poses[0].position.y = 0.32;
    bin1_object.mesh_poses[0].position.z = 0.75;
    quaternionTFToMsg(tf::createQuaternionFromRPY(0, -0.19, 3.16), bin1_object.mesh_poses[0].orientation);  // set orientation of bin1


    bin1_object.meshes.push_back(mesh);
    bin1_object.mesh_poses.push_back(bin1_object.mesh_poses[0]);
    bin1_object.operation = bin1_object.ADD;
    collision_object_pub.publish(bin1_object);


    ///////////////// Add objects to moveit world /////////////////
    std::vector<moveit_msgs::CollisionObject> collision_vector;
    collision_vector.push_back(container_object);
    collision_vector.push_back(belt_object);
    collision_vector.push_back(bin1_object);

    planning_scene_interface.addCollisionObjects(collision_vector);
    ROS_INFO("shipping_container, belt and bin1 added into the world");
  }

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

  void first_task(){

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.startStateMonitor(6.0);
    //const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // initialize rviz and clear all markers
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    visual_tools.deleteAllMarkers();

    // load rviz control
    visual_tools.loadRemoteControl();

    //loadObstacles(move_group, planning_scene_interface);
    loadMeshObstacles(move_group, planning_scene_interface);

    // update rviz with obstacles
    visual_tools.trigger();
    ros::Duration(3.0).sleep();   // wait for rviz to update

    //move_group.setStartState(*move_group.getCurrentState());

    //ROS_INFO("Reference frame: ");
    //ROS_INFO(move_group.getPlanningFrame().c_str());
    //ROS_INFO("End effector link: ");
    //ROS_INFO(move_group.getEndEffectorLink().c_str());

    //ROS_INFO("Printing camera object poses");
    //for( int i = 0; i < piston_part_pose.size(); ++i ){
      //ROS_INFO( piston_part_pose[i].c_str() );
    //}
/*
    tf::Transformer listener;
    tf::Transform transform;
    try{
      listener.lookupTransform("/world", "/logical_camera_1_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
*/

    // set target pose
    geometry_msgs::Pose target_pose1, target_pose2; // = piston_part_pose.back();
    //piston_part_pose.pop_back();
    // grab gasket part top-left
    tf::Quaternion q;
    q = tf::createQuaternionFromRPY(0,2.37,0);
    //target_pose1.orientation.w = 0.0;
    //target_pose1.orientation.x = 0.0;
    //target_pose1.orientation.y = 0.0;
    //target_pose1.orientation.z = 0.0;
    //target_pose1.orientation.w = q.w;
    //target_pose1.orientation.x = q.x;
    //target_pose1.orientation.y = q.y;
    //target_pose1.orientation.z = q.z;
    quaternionTFToMsg(q, target_pose1.orientation);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = 0.8;

    move_group.setPoseTarget(target_pose1);
    ROS_INFO("Target pose set");

    // path planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan, my_plan2;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal)");
    if(success)
      ROS_INFO("PASSED");
    else
      ROS_INFO("FAILED");

    // publish msg to gazebo
    //joint_trajectory_publisher_.publish(my_plan.trajectory_.joint_trajectory);
    
    ROS_INFO("Visualizing plan 1 as trajectory line");

    // publish trajectory line to rviz
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    
    // visualize planned path
    visual_tools.trigger();
    //visual_tools.prompt("next step");

    ros::Duration(5.0).sleep();

    move_group.setStartState(*move_group.getCurrentState());

    q = tf::createQuaternionFromRPY(0,-2.37,0);
    quaternionTFToMsg(q, target_pose2.orientation);
    target_pose2.position.x = -0.9;
    target_pose2.position.y = 0.2;
    target_pose2.position.z = 0.9;

    move_group.setPoseTarget(target_pose2);

    success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal)");
    if(success)
      ROS_INFO("PASSED");
    else
      ROS_INFO("FAILED");

    // publish msg to gazebo
    //joint_trajectory_publisher_.publish(my_plan2.trajectory_.joint_trajectory);
    
    ROS_INFO("Visualizing plan 1 as trajectory line");

    // publish trajectory line to rviz
    //visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
    
    // visualize planned path
    visual_tools.trigger();

    //ROS_INFO("Attach the object to the robot");
    //move_group.attachObject(collision_object.id);

    // move real robot to target
    //move_group.move();
  }
  
private:
  ros::Subscriber current_score_subscriber;
  ros::Subscriber competition_state_subscriber;
  ros::Subscriber orders_subscriber;
  ros::Subscriber joint_state_subscriber;
  ros::Subscriber break_beam_subscriber;
  ros::Subscriber logical_camera_subscriber;

  ros::Publisher joint_trajectory_publisher_;
  ros::Publisher collision_object_pub;
  
  std::string                     competition_state_;
  double                          current_score_;
  std::vector<osrf_gear::Order>   received_orders_;
  sensor_msgs::JointState         current_joint_states_;
  bool                            has_been_zeroed_;

  //////////////////////////////
  std::vector<geometry_msgs::Pose> piston_part_pose;


};


int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "vs_qual_node");
  ros::NodeHandle node;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);    // everything happens here

  ROS_INFO("Setup complete.");
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
