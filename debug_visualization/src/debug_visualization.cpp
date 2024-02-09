/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/debug_visualization.h"

const DebugVisualization::NodeGeometry DebugVisualization::NODE_GEOMETRY_[7] = 
{
  NodeGeometry(FPoint2D(0.f + X_GLOBAL_OFFSET, 0.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),     //camera
  NodeGeometry(FPoint2D(0.f + X_GLOBAL_OFFSET, -2.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),    //lidar
  NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -1.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),    //fusion
  NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -3.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),    //slam
  NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -5.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),    //pathPlanning
  NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -7.f + Y_GLOBAL_OFFSET), 2.5f, 1.f),    //pathTracking
  NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -9.f + Y_GLOBAL_OFFSET), 2.5f, 1.f)     //jetsonCanInterface
};

DebugVisualization::NodeGeometry::NodeGeometry(const FPoint2D &position, float scale_x, float scale_y)
{
  this->position = position;
  this->scale_x = scale_x;
  this->scale_y = scale_y;
}

geometry_msgs::Point DebugVisualization::FPoint2D::getPoint() const
{
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = 0.;

  return point;
}

DebugVisualization::DebugVisualization(ros::NodeHandle& nh) :
  /* ROS interface init */
  publisher_(nh.advertise<visualization_msgs::MarkerArray>("debug_visualization_out", 1)),
  camera_sub_(nh.subscribe("camera_cone_detection_debug_state", 2, &DebugVisualization::cameraCallback, this)),

  lidar_sub_(nh.subscribe("lidar_cone_detection_debug_state", 2, &DebugVisualization::lidarCallback, this)),
  fusion_sub_(nh.subscribe("fusion_debug_state", 2, &DebugVisualization::fusionCallback, this)),
  slam_sub_(nh.subscribe("slam_debug_state", 2, &DebugVisualization::slamCallback, this)),
  path_planning_sub_(nh.subscribe("pathplanning_debug_state", 2, &DebugVisualization::pathPlanningCallback, this)),
  path_tracking_sub_(nh.subscribe("pathtracking_debug_state", 2, &DebugVisualization::pathTrackingCallback, this)),
  jetson_CAN_interface_sub_(nh.subscribe("jetson_can_interface_debug_state", 2, &DebugVisualization::jetsonCANInterfaceCallback, this))
{
  initMarkerArray();
  initNodes();
  initConnectionLines();
  initNodeNames();
  initNodeFrequency();
  initNodeWorkTime();
  initNodeOutputs();
}

void DebugVisualization::initConnectionLines()
{
  for(int32_t i = 0; i < NUM_OF_CONNECTION_LINES; i++)
  {
    connection_lines_[i].header.frame_id = FRAME_ID;
    connection_lines_[i].ns = LINE_CONNECTION_NAMESPACE;
    connection_lines_[i].id = i;
    connection_lines_[i].action = visualization_msgs::Marker::ADD;

    connection_lines_[i].type = visualization_msgs::Marker::ARROW;
    connection_lines_[i].scale.x = 0.08f;
    connection_lines_[i].scale.y = 0.08f;
    connection_lines_[i].scale.z = 0.5f;

    connection_lines_[i].pose.orientation.w = 1.f;
    connection_lines_[i].pose.orientation.x = 0.f;
    connection_lines_[i].pose.orientation.y = 0.f;
    connection_lines_[i].pose.orientation.z = 0.f;

    connection_lines_[i].color.a = 1.f;
    connection_lines_[i].color.r = 0.5f;
    connection_lines_[i].color.g = 0.5f;
    connection_lines_[i].color.b = 0.5f;

    connection_lines_[i].pose.position = FPoint2D(0.f, 0.f).getPoint();

    connection_lines_[i].points.reserve(2);
    connection_lines_[i].points.push_back(NODE_GEOMETRY_[i].position.getPoint());
  }

  connection_lines_[CAMERA].points.push_back(NODE_GEOMETRY_[FUSION].position.getPoint());   
  connection_lines_[LIDAR].points.push_back(NODE_GEOMETRY_[FUSION].position.getPoint());  
  connection_lines_[FUSION].points.push_back(NODE_GEOMETRY_[SLAM].position.getPoint());
  connection_lines_[SLAM].points.push_back(NODE_GEOMETRY_[PATH_PLANNING].position.getPoint());   
  connection_lines_[PATH_PLANNING].points.push_back(NODE_GEOMETRY_[PATH_TRACKING].position.getPoint());
  connection_lines_[PATH_TRACKING].points.push_back(NODE_GEOMETRY_[JETSON_CAN_INTERFACE].position.getPoint());
}

void DebugVisualization::initNodes()
{
  for(int32_t i = 0; i < NUM_OF_NODES; i++)
  {
    node_markers_[i].header.frame_id = FRAME_ID;
    node_markers_[i].ns = NODE_RECT_NAMESPACE;
    node_markers_[i].id = i;
    node_markers_[i].action = visualization_msgs::Marker::ADD;

    node_markers_[i].type = visualization_msgs::Marker::CUBE;
    node_markers_[i].scale.x = NODE_GEOMETRY_[i].scale_x;
    node_markers_[i].scale.y = NODE_GEOMETRY_[i].scale_y;
    node_markers_[i].scale.z = 0.1f;

    node_markers_[i].color.a = 1.f;
    node_markers_[i].color.r = 0.3f;
    node_markers_[i].color.g = 0.3f;
    node_markers_[i].color.b = 0.3f;

    node_markers_[i].pose.position = NODE_GEOMETRY_[i].position.getPoint();
    node_markers_[i].pose.orientation.w = 1.f;
    node_markers_[i].pose.orientation.x = 0.f;
    node_markers_[i].pose.orientation.y = 0.f;
    node_markers_[i].pose.orientation.z = 0.f;
  }
}

void DebugVisualization::initNodeOutputs()
{
  for(int32_t i = 0; i < NUM_OF_NODES; i++)
  {
    node_outputs_[i].header.frame_id = FRAME_ID;
    node_outputs_[i].ns = OUTPUTS_NAMESPACE;
    node_outputs_[i].id = i;
    node_outputs_[i].action = visualization_msgs::Marker::ADD;

    node_outputs_[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_outputs_[i].scale.z = 0.5f;

    node_outputs_[i].pose.orientation.w = 1.f;
    node_outputs_[i].pose.orientation.x = 0.f;
    node_outputs_[i].pose.orientation.y = 0.f;
    node_outputs_[i].pose.orientation.z = 0.f;

    node_outputs_[i].color.a = 1.f;
    node_outputs_[i].color.r = 1.f;
    node_outputs_[i].color.g = 1.f;
    node_outputs_[i].color.b = 1.f;

    node_outputs_[i].pose.position.x = NODE_GEOMETRY_[i].position.getPoint().x + 0.5f;
    node_outputs_[i].pose.position.y = NODE_GEOMETRY_[i].position.getPoint().y - 0.5f;
    node_outputs_[i].pose.position.z = 1.f;
    node_outputs_[i].text = "OUTPUT";
  }
}

void DebugVisualization::printError(const char* NODE) const
{
  std::cout << "ERROR: Unknown debug state received - " << NODE << std::endl;
}

void DebugVisualization::update(const sgtdv_msgs::DebugState::ConstPtr &msg, NODE_TYPE type)
{
  if (msg->working_state == 1)
  {
    setMarkerColorRed(type);
    start_time_[type] = msg->stamp;
    started_[type] = true;
  }
  else if (msg->working_state == 0)
  {
    std::stringstream ss;
    setMarkerColorGreen(type);
    auto work_load_time = (ros::Duration(msg->stamp - start_time_[type]).toNSec() / 1e6);
    
    if (!started_[type]) work_load_time = 0;

    ss << std::fixed << std::setprecision(2) << work_load_time << " ms";
    node_work_time_[type].text = ss.str();
    ss.str("");

    const auto time_since_last_run = (ros::Duration(msg->stamp - end_time_[type]).toNSec() / 1e6);
    const float seconds = time_since_last_run / 1000.f;
    ss << std::fixed << std::setprecision(2) << 1.f / seconds << " Hz";
    node_frequency_[type].text = ss.str();
    end_time_[type] = msg->stamp;
    ss.str("");

    switch(type)
    {
      case CAMERA:
      case LIDAR:
      case FUSION:
      case SLAM:
        ss << msg->num_of_cones << " cones";
        break;

      case PATH_PLANNING:
        ss << msg->num_of_cones << " points";
        break;
          
      case PATH_TRACKING:
      case JETSON_CAN_INTERFACE:
        ss << "Speed: " << msg->speed << " Angle: " << msg->angle;
        break;
      default: break;
    }

    node_outputs_[type].text = ss.str();
    started_[type] = false;
  }
  else
  {
    printError(NODE_TYPE_STRINGS[type]);
    return;
  }

  const ros::Time now = ros::Time::now();

  node_markers_[type].header.stamp = now;
  connection_lines_[type].header.stamp = now;
  node_work_time_[type].header.stamp = now;
  node_frequency_[type].header.stamp = now;
  node_outputs_[type].header.stamp = now;
}

void DebugVisualization::cameraCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, CAMERA);
}

void DebugVisualization::lidarCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, LIDAR);
}

void DebugVisualization::fusionCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, FUSION);
}

void DebugVisualization::slamCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, SLAM);
}

void DebugVisualization::pathPlanningCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, PATH_PLANNING);
}

void DebugVisualization::pathTrackingCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, PATH_TRACKING);
}

void DebugVisualization::jetsonCANInterfaceCallback(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
  update(msg, JETSON_CAN_INTERFACE);
}

void DebugVisualization::setMarkerColor(const float r, const float g, const float b, const NODE_TYPE type)
{
  node_markers_[type].color.r = r;
  node_markers_[type].color.g = g;
  node_markers_[type].color.b = b;
}

void DebugVisualization::setMarkerColorRed(const NODE_TYPE type)
{
  setMarkerColor(1.f, 0.f, 0.f, type);
}

void DebugVisualization::setMarkerColorGreen(const NODE_TYPE type)
{
  setMarkerColor(0.f, 1.f, 0.f, type);
}

void DebugVisualization::initNodeNames()
{
  for(int32_t i = 0; i < NUM_OF_NODES; i++)
  {
    node_names_[i].header.frame_id = FRAME_ID;
    node_names_[i].ns = NAMES_NAMESPACE;
    node_names_[i].id = i;
    node_names_[i].action = visualization_msgs::Marker::ADD;

    node_names_[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_names_[i].scale.z = 0.5f;

    node_names_[i].pose.orientation.w = 1.f;
    node_names_[i].pose.orientation.x = 0.f;
    node_names_[i].pose.orientation.y = 0.f;
    node_names_[i].pose.orientation.z = 0.f;

    node_names_[i].color.a = 1.f;
    node_names_[i].color.r = 1.f;
    node_names_[i].color.g = 1.f;
    node_names_[i].color.b = 1.f;

    node_names_[i].pose.position = NODE_GEOMETRY_[i].position.getPoint();
    node_names_[i].pose.position.z = 1.f;
    node_names_[i].text = NODE_TYPE_STRINGS[i];
  }
}

void DebugVisualization::initNodeFrequency()
{
  for(int32_t i = 0; i < NUM_OF_NODES; i++)
  {
    node_frequency_[i].header.frame_id = FRAME_ID;
    node_frequency_[i].ns = FREQUENCY_NAMESPACE;
    node_frequency_[i].id = i;
    node_frequency_[i].action = visualization_msgs::Marker::ADD;

    node_frequency_[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_frequency_[i].scale.z = 0.5f;

    node_frequency_[i].pose.orientation.w = 1.f;
    node_frequency_[i].pose.orientation.x = 0.f;
    node_frequency_[i].pose.orientation.y = 0.f;
    node_frequency_[i].pose.orientation.z = 0.f;

    node_frequency_[i].color.a = 1.f;
    node_frequency_[i].color.r = 0.f;
    node_frequency_[i].color.g = 1.f;
    node_frequency_[i].color.b = 0.f;

    FPoint2D buff;
    buff.x = NODE_GEOMETRY_[i].position.x + 1.f;
    buff.y = NODE_GEOMETRY_[i].position.y + 0.5f;

    node_frequency_[i].pose.position = buff.getPoint();
    node_frequency_[i].pose.position.z = 1.f;
    node_frequency_[i].text = "0 Hz";
    end_time_[i] = ros::Time::now();
  }
}

void DebugVisualization::initNodeWorkTime()
{
  for(int32_t i = 0; i < NUM_OF_NODES; i++)
  {
    node_work_time_[i].header.frame_id = FRAME_ID;
    node_work_time_[i].ns = WORKTIME_NAMESPACE;
    node_work_time_[i].id = i;
    node_work_time_[i].action = visualization_msgs::Marker::ADD;

    node_work_time_[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_work_time_[i].scale.z = 0.5f;

    node_work_time_[i].pose.orientation.w = 1.f;
    node_work_time_[i].pose.orientation.x = 0.f;
    node_work_time_[i].pose.orientation.y = 0.f;
    node_work_time_[i].pose.orientation.z = 0.f;

    node_work_time_[i].color.a = 1.f;
    node_work_time_[i].color.r = 1.f;
    node_work_time_[i].color.g = 0.f;
    node_work_time_[i].color.b = 0.f;

    FPoint2D buff;
    buff.x = NODE_GEOMETRY_[i].position.x - 1.f;
    buff.y = NODE_GEOMETRY_[i].position.y + 0.5f;

    node_work_time_[i].pose.position = buff.getPoint();
    node_work_time_[i].pose.position.z = 1.f;
    node_work_time_[i].text = "0 ms";
    start_time_[i] = ros::Time::now();
  }
}

void DebugVisualization::initMarkerArray()
{
  marker_array_.markers.resize(5 * NUM_OF_NODES + NUM_OF_CONNECTION_LINES);

  connection_lines_ = &(marker_array_.markers[0]);
  node_markers_ = connection_lines_ + NUM_OF_CONNECTION_LINES;
  node_names_ = node_markers_ + NUM_OF_NODES;
  node_frequency_ = node_names_ + NUM_OF_NODES;
  node_work_time_ = node_frequency_ + NUM_OF_NODES;
  node_outputs_ = node_work_time_ + NUM_OF_NODES;
}

void DebugVisualization::publishEverythingAsArray() const
{
  publisher_.publish(marker_array_);
}