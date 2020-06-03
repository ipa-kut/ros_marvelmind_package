#ifndef MARVELMIND_NAVIGATION_H
#define MARVELMIND_NAVIGATION_H

#include <fcntl.h>
#include <iostream>
#include <semaphore.h>
#include <sstream>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "marvelmind_interfaces/msg/hedge_pos.hpp"
#include "marvelmind_interfaces/msg/hedge_pos_a.hpp"
#include "marvelmind_interfaces/msg/hedge_pos_ang.hpp"
#include "marvelmind_interfaces/msg/beacon_pos_a.hpp"
#include "marvelmind_interfaces/msg/hedge_imu_raw.hpp"
#include "marvelmind_interfaces/msg/hedge_imu_fusion.hpp"
#include "marvelmind_interfaces/msg/beacon_distance.hpp"
#include "marvelmind_interfaces/msg/hedge_telemetry.hpp"
#include "marvelmind_interfaces/msg/hedge_quality.hpp"
#include "marvelmind_interfaces/msg/marvelmind_waypoint.hpp"
#include "std_msgs/msg/string.hpp"
extern "C"
{
#include "marvelmind_nav/marvelmind_hedge.h"
}


#define ROS_NODE_NAME "hedge_rcv_bin"
#define HEDGE_POSITION_TOPIC_NAME "hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "hedge_pos_a"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "hedge_pos_ang"
#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "beacons_pos_a"
#define HEDGE_IMU_RAW_TOPIC_NAME "hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "hedge_imu_fusion"
#define BEACON_RAW_DISTANCE_TOPIC_NAME "beacon_raw_distance"
#define HEDGE_TELEMETRY_TOPIC_NAME "hedge_telemetry"
#define HEDGE_QUALITY_TOPIC_NAME "hedge_quality"
#define MARVELMIND_WAYPOINT_TOPIC_NAME "marvelmind_waypoint"



class MarvelmindNavigation : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MarvelmindNavigation(const std::string & node_name,int argc, char **argv, bool intra_process_comms=false)
  : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
      argc_(argc),argv_(argv)
{}
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
~MarvelmindNavigation() {}

private:
  int argc_;
  char **argv_;
  std::shared_ptr<rclcpp::TimerBase> timer_;

  struct MarvelmindHedge * hedge= NULL;
  struct timespec ts;

  static uint32_t hedge_timestamp_prev;
//  static sem_t *sem;

  marvelmind_interfaces::msg::HedgePos hedge_pos_noaddress_msg;// hedge coordinates message (old version without address) for publishing to ROS topic
  marvelmind_interfaces::msg::HedgePosA hedge_pos_msg;// hedge coordinates message for publishing to ROS topic
  marvelmind_interfaces::msg::HedgePosAng hedge_pos_ang_msg;// hedge coordinates and angle message for publishing to ROS topic
  marvelmind_interfaces::msg::BeaconPosA beacon_pos_msg;// stationary beacon coordinates message for publishing to ROS topic
  marvelmind_interfaces::msg::HedgeImuRaw hedge_imu_raw_msg;// raw IMU data message for publishing to ROS topic
  marvelmind_interfaces::msg::HedgeImuFusion hedge_imu_fusion_msg;// IMU fusion data message for publishing to ROS topic
  marvelmind_interfaces::msg::BeaconDistance beacon_raw_distance_msg;// Raw distance message for publishing to ROS topic
  marvelmind_interfaces::msg::HedgeTelemetry hedge_telemetry_msg;// Telemetry message for publishing to ROS topic
  marvelmind_interfaces::msg::HedgeQuality hedge_quality_msg;// Quality message for publishing to ROS topic
  marvelmind_interfaces::msg::MarvelmindWaypoint marvelmind_waypoint_msg;// Waypoint message for publishing to ROS topic

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgePosAng>> hedge_pos_ang_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgePosA>> hedge_pos_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgePos>> hedge_pos_noaddress_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::BeaconPosA>> beacons_pos_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgeImuRaw>> hedge_imu_raw_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgeImuFusion>> hedge_imu_fusion_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::BeaconDistance>> beacon_distance_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgeTelemetry>> hedge_telemetry_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::HedgeQuality>> hedge_quality_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<marvelmind_interfaces::msg::MarvelmindWaypoint>> marvelmind_waypoint_publisher_;



//  Marvelmind internal functions
  int hedgeReceivePrepare();
  bool hedgeReceiveCheck();
  bool beaconReceiveCheck();
  bool hedgeIMURawReceiveCheck();
  bool hedgeIMUFusionReceiveCheck();
  void getRawDistance(uint8_t index);
  bool hedgeTelemetryUpdateCheck();
  bool hedgeQualityUpdateCheck();
  bool marvelmindWaypointUpdateCheck();

// LC node functions
  void activateAllPublishers();
  void createPublishers();
  void deactivateAllPublishers();
  void resetAllPublishers();
  void setMessageDefaults();

  void main_loop();

};



#endif // MARVELMIND_NAVIGATION_H
