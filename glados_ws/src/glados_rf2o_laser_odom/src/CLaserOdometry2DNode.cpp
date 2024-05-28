/** ****************************************************************************************
 *  This is Usames version of the rf2o library
 *  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
 *  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es
 *
 * Modifications: Jeremie Deray & (see contributons on github)
 ******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
using namespace std::chrono_literals;
using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode() : Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);

  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->get_parameter("base_frame_id", base_frame_id);

  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);

  // TF Stuff
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // ODOM Publishers
  local_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // Subscriptions

  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
                                                                     std::bind(&CLaserOdometry2DNode::laserCallBack, this, std::placeholders::_1));

  xbox_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&CLaserOdometry2DNode::resetOdomCallBack, this, std::placeholders::_1));

  // Set total odom to 0
  globalOdom.x = 0;
  globalOdom.y = 0;
  globalOdom.theta = 0;

  // Initialize pose
  zeroOut();

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan = true;
}

void CLaserOdometry2DNode::zeroOut()
{
  // init to 0
  globalOdom.x = rf2o_ref.robot_pose_.translation()(0);
  globalOdom.y = rf2o_ref.robot_pose_.translation()(1);
  globalOdom.theta = rf2o::getYaw(rf2o_ref.robot_pose_.rotation());

  GT_pose_initialized = true;
  initial_robot_pose.pose.pose.position.x = globalOdom.x;
  initial_robot_pose.pose.pose.position.y = globalOdom.y;
  initial_robot_pose.pose.pose.position.z = 0;

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, globalOdom.theta);
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);

  initial_robot_pose.pose.pose.orientation.w = quaternion.w;
  initial_robot_pose.pose.pose.orientation.x = quaternion.x;
  initial_robot_pose.pose.pose.orientation.y = quaternion.y;
  initial_robot_pose.pose.pose.orientation.z = quaternion.z;
}

void CLaserOdometry2DNode::resetOdom()
{
  zeroOut();
  rf2o_ref.module_initialized = false;
  RCLCPP_INFO(get_logger(), "Odometry reset");

  setLaserPoseFromTf();
  rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
  rf2o_ref.first_laser_scan = false;
}

void CLaserOdometry2DNode::resetOdomCallBack(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[2] == 1) // X Button
  {
    resetOdom();
  }
}

void CLaserOdometry2DNode::laserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    // Keep in memory the last received laser_scan
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;

    if (rf2o_ref.first_laser_scan == false)
    {
      // copy laser range data to rf2o internal variable
      for (unsigned int i = 0; i < rf2o_ref.width; i++)
        rf2o_ref.range_wf(i) = new_scan->ranges[i];
      // inform of new scan available
      new_scan_available = true;
    }
    else
    {
      resetOdom();
    }
  }
}

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero, 150ms);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    retrieved = false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      R(r, c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process()
{
  if (rf2o_ref.is_initialized() && scan_available())
  {
    // Process odometry estimation
    rf2o_ref.odometryCalculation(last_scan);

    local_publish();

    // Do not run on the same data!
    new_scan_available = false;
  }
}

/* ODOM PUBLISHING */

void CLaserOdometry2DNode::local_publish()
{

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time; // the time of the last scan used!
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;

  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed; // linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed; // angular speed

  local_odom_pub->publish(odom);

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = rf2o_ref.last_odom_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
  odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = quaternion;

  odom_broadcaster->sendTransform(odom_trans);
}

/* MAIN */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok())
  {
    rclcpp::spin_some(myLaserOdomNode);
    myLaserOdomNode->process();
    rate.sleep();
  }

  return 0;
}