/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

/* C++ */
#include <string>
#include <sstream>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* SGT */
#include <sgtdv_msgs/DebugState.h>

constexpr int32_t NUM_OF_CONNECTION_LINES = 6;
constexpr int32_t NUM_OF_NODES = 7;

#define ENUM_MACRO(name, v1, v2, v3, v4, v5, v6, v7)\
  enum name { v1 = 0, v2, v3, v4, v5, v6, v7};\
  const char *name##_STRINGS[NUM_OF_NODES] = { #v1, #v2, #v3, #v4, #v5, #v6, #v7};

constexpr const char* FRAME_ID = "sgt_frame";
constexpr const char* NODE_RECT_NAMESPACE = "nodeGeometry";
constexpr const char* LINE_CONNECTION_NAMESPACE = "lineConnections";
constexpr const char* NAMES_NAMESPACE = "nodeNames";
constexpr const char* FREQUENCY_NAMESPACE = "nodeFrequency";
constexpr const char* WORKTIME_NAMESPACE = "nodeWorkTime";
constexpr const char* OUTPUTS_NAMESPACE = "nodeOutputs";
constexpr const float X_GLOBAL_OFFSET = -2.f;
constexpr const float Y_GLOBAL_OFFSET = 2.f;

class DebugVisualization
{

ENUM_MACRO(NODE_TYPE, CAMERA, LIDAR, FUSION, SLAM, PATH_PLANNING, PATH_TRACKING, JETSON_CAN_INTERFACE)


struct FPoint2D
{
  FPoint2D() { x = 0.f; y = 0.f; }
  FPoint2D(float X, float Y) {x = X; y = Y;}
  geometry_msgs::Point getPoint() const;
  float x;
  float y;
};

struct NodeGeometry
{
  NodeGeometry(const FPoint2D &position, float scale_x, float scale_y);
  FPoint2D position;
  float scale_x;
  float scale_y;
};

public:
  DebugVisualization(ros::NodeHandle& nh);
  ~DebugVisualization() = default;
  
  void cameraCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void lidarCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void fusionCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void slamCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void pathPlanningCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void pathTrackingCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
  void jetsonCANInterfaceCallback(const sgtdv_msgs::DebugState::ConstPtr &msg);
   
  void publishEverythingAsArray(void) const;

private:
  ros::Publisher publisher_;
  ros::Subscriber camera_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber fusion_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber path_planning_sub_;
  ros::Subscriber path_tracking_sub_;
  ros::Subscriber jetson_CAN_interface_sub_;

  visualization_msgs::Marker *connection_lines_ = nullptr;
  visualization_msgs::Marker *node_markers_ = nullptr;
  visualization_msgs::Marker *node_names_ = nullptr;
  visualization_msgs::Marker *node_frequency_ = nullptr;
  visualization_msgs::Marker *node_work_time_ = nullptr;
  visualization_msgs::Marker *node_outputs_ = nullptr;
  visualization_msgs::MarkerArray marker_array_;
  bool started_[NUM_OF_NODES] = { false, false, false, false, false, false, false };

  ros::Time start_time_[NUM_OF_NODES];
  ros::Time end_time_[NUM_OF_NODES];

  void update(const sgtdv_msgs::DebugState::ConstPtr &msg, NODE_TYPE type);
  void initMarkerArray(void);
  void initConnectionLines(void);  
  void initNodes(void);
  void initNodeNames(void);
  void initNodeFrequency(void);
  void initNodeWorkTime(void);
  void initNodeOutputs(void);

  void printError(const char* NODE) const;
  void setMarkerColor(const float r, const float g, const float b, const NODE_TYPE type);
  void setMarkerColorRed(const NODE_TYPE type);
  void setMarkerColorGreen(const NODE_TYPE type);

  static const NodeGeometry NODE_GEOMETRY_[NUM_OF_NODES];
};
  