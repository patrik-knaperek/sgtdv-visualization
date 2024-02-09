/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/visualizator.h"

Visualizator::Visualizator(ros::NodeHandle& handle) :
  /* ROS interface init */
  camera_publisher_(handle.advertise<visualization_msgs::MarkerArray>("camera_cones/marker", 1)),
  lidar_publisher_(handle.advertise<visualization_msgs::MarkerArray>("lidar_cones/marker",1)),
  fusion_publisher_(handle.advertise<visualization_msgs::MarkerArray>("fusion_cones/marker",1)),
  pose_publisher_(handle.advertise<visualization_msgs::Marker>("slam/pose/marker", 1)),
  map_publisher_(handle.advertise<visualization_msgs::Marker>("slam/map/marker", 1)),
  trajectory_publisher_(handle.advertise<visualization_msgs::Marker>("pathplanning_trajectory/marker",1, true)),
  command_publisher_(handle.advertise<visualization_msgs::MarkerArray>("pathtracking_commands/marker", 1)),
  camera_fov_publisher_(handle.advertise<geometry_msgs::PolygonStamped>("camera/fov_visualize", 1, true)),
  lidar_fov_publisher_(handle.advertise<geometry_msgs::PolygonStamped>("lidar/fov_visualize", 1, true)),

  camera_subscriber_(handle.subscribe("camera_cones", 1, &Visualizator::cameraCallback, this)),
  lidar_subscriber_(handle.subscribe("lidar_cones", 1, &Visualizator::lidarCallback, this)),
  fusion_subscriber_(handle.subscribe("fusion_cones", 1, &Visualizator::fusionCallback, this)),
  pose_subscriber_(handle.subscribe("slam/pose", 1, &Visualizator::poseCallback, this)),
  map_subscriber_(handle.subscribe("slam/map", 1, &Visualizator::mapCallback, this)),
  trajectory_subscriber_(handle.subscribe("pathplanning_trajectory", 1, &Visualizator::trajectoryCallback, this)),
  command_subscriber_(handle.subscribe("pathtracking_commands", 1, &Visualizator::commandCallback, this))
{
  initCameraMarker();
  initLidarMarker();
  initFusionMarker();
  initPoseMarker();
  initMapMarker();
  initTrajectoryMarker();
  initCommandMarkers(handle);
  initFOV(handle);
}

void Visualizator::initCameraMarker(void)
{
  camera_marker_.type = visualization_msgs::Marker::SPHERE;
  camera_marker_.action = visualization_msgs::Marker::ADD;
  camera_marker_.lifetime = ros::Duration(0.5);
  camera_marker_.pose.orientation.w = 1.0;
  camera_marker_.scale.x = 0.1;
  camera_marker_.scale.y = 0.1;
  camera_marker_.scale.z = 0.1;
  camera_marker_.color.a = 0.6;
}

void Visualizator::initLidarMarker(void)
{  
  lidar_marker_.type = visualization_msgs::Marker::CYLINDER;
  lidar_marker_.action = visualization_msgs::Marker::ADD;
  lidar_marker_.lifetime = ros::Duration(0.5);
  lidar_marker_.pose.orientation.w = 1.0;
  lidar_marker_.scale.x = 0.1;
  lidar_marker_.scale.y = 0.1;
  lidar_marker_.scale.z = 0.1;
  lidar_marker_.color.a = 0.7;
  lidar_marker_.color.r = 1.0;
}

void Visualizator::initFusionMarker(void)
{
  fusion_marker_.type = visualization_msgs::Marker::CUBE;
  fusion_marker_.action = visualization_msgs::Marker::ADD;
  fusion_marker_.lifetime = ros::Duration(0.5);
  fusion_marker_.pose.orientation.w = 1.0;
  fusion_marker_.scale.x = 0.1;
  fusion_marker_.scale.y = 0.1;
  fusion_marker_.scale.z = 0.1;
  fusion_marker_.color.a = 1.0;
}

void Visualizator::initPoseMarker(void)
{
  pose_marker_.header.frame_id =  "map";
  pose_marker_.header.stamp = ros::Time();
  pose_marker_.type = visualization_msgs::Marker::POINTS;
  pose_marker_.action = visualization_msgs::Marker::ADD;
  pose_marker_.lifetime = ros::Duration(0);
  pose_marker_.scale.x = 0.3;
  pose_marker_.scale.y = 0.3;
  pose_marker_.color.a = 1;
  pose_marker_.color.r = 1;
}

void Visualizator::initMapMarker(void)
{
  map_marker_.header.frame_id = "map";
  map_marker_.type = visualization_msgs::Marker::POINTS;
  map_marker_.action = visualization_msgs::Marker::ADD;
  map_marker_.scale.x = 0.2;
  map_marker_.scale.y = 0.2;
}

void Visualizator::initTrajectoryMarker(void)
{
  trajectory_marker_.type = visualization_msgs::Marker::POINTS;
  trajectory_marker_.header.frame_id = "map";
  trajectory_marker_.scale.x = 0.2;
  trajectory_marker_.scale.y = 0.2;
  trajectory_marker_.color.r = 1.0f;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.pose.orientation.w = 1.0;
}

void Visualizator::initCommandMarkers(const ros::NodeHandle& handle)
{
  /* Load command parameters from server */
  
  double steer_min, steer_max;
  int throttle_min, throttle_max;
  std::string base_frame_id;
  Utils::loadParam(handle, "/controller/steering/min", -1.0, &steer_min);
  Utils::loadParam(handle, "/controller/steering/max", 1.0, &steer_max);
  Utils::loadParam(handle, "/controller/speed/min", 0, &throttle_min);
  Utils::loadParam(handle, "/controller/speed/max", 0, &throttle_max);
  Utils::loadParam(handle, "/base_frame_id", std::string("base_link"), &base_frame_id);

  geometry_msgs::Point point;
  
  /* Init throttle command marker parameters */

  visualization_msgs::Marker throttle_range_marker;
  throttle_range_marker.points.reserve(2);
  throttle_range_marker.header.frame_id = base_frame_id;
  throttle_range_marker.type = visualization_msgs::Marker::LINE_STRIP;
  throttle_range_marker.action = visualization_msgs::Marker::ADD;
  throttle_range_marker.id = 0;
  throttle_range_marker.ns = "throttle range";
  throttle_range_marker.color.r = 1.0;
  throttle_range_marker.color.a = 0.2;
  throttle_range_marker.scale.x = 0.15;
  throttle_range_marker.scale.y = 0.1;
  throttle_range_marker.scale.z = 0.1;
  throttle_range_marker.pose.orientation.w = 1;
  throttle_range_marker.pose.position.z = 0.2;
  
  point.x = THROTTLE_MARKER_BASE[0] + throttle_min * THROTTLE_GAIN;
  point.y = THROTTLE_MARKER_BASE[1];
  throttle_range_marker.points.push_back(point);
  point.x = THROTTLE_MARKER_BASE[0] + throttle_max * THROTTLE_GAIN;
  throttle_range_marker.points.push_back(point);

  visualization_msgs::Marker throttle_marker;
  throttle_marker.points.reserve(2);
  throttle_marker.header.frame_id = base_frame_id;
  throttle_marker.type = visualization_msgs::Marker::LINE_STRIP;
  throttle_marker.action = visualization_msgs::Marker::ADD;
  throttle_marker.id = 2;
  throttle_marker.ns = "throttle cmd";
  throttle_marker.color.g = 1.0;
  throttle_marker.color.a = 1.0;
  throttle_marker.scale.x = 0.15;
  throttle_marker.scale.y = 0.1;
  throttle_marker.scale.z = 0.1;
  throttle_marker.pose.orientation.w = 1;
  throttle_marker.pose.position.z = 0.25;
  
  point.x = THROTTLE_MARKER_BASE[0];
  point.y = THROTTLE_MARKER_BASE[1];
  throttle_marker.points.push_back(point);
  throttle_marker.points.push_back(point);
  
  /* Init steering command marker parameters */
  
  visualization_msgs::Marker steer_range_marker;
  steer_range_marker.points.reserve(2);
  steer_range_marker.header.frame_id = base_frame_id;
  steer_range_marker.type = visualization_msgs::Marker::LINE_STRIP;
  steer_range_marker.action = visualization_msgs::Marker::ADD;
  steer_range_marker.id = 1;
  steer_range_marker.ns = "steering range";
  steer_range_marker.color.r = 1.0;
  steer_range_marker.color.a = 0.2;
  steer_range_marker.scale.x = 0.15;
  steer_range_marker.scale.y = 0.1;
  steer_range_marker.scale.z = 0.1;
  steer_range_marker.pose.orientation.w = 1;
  steer_range_marker.pose.position.z = 0.2;
  
  point.x = STEER_MARKER_BASE[0];
  point.y = STEER_MARKER_BASE[1] + steer_min * STEER_GAIN;
  steer_range_marker.points.push_back(point);
  point.y = STEER_MARKER_BASE[1] + steer_max * STEER_GAIN;
  steer_range_marker.points.push_back(point);

  visualization_msgs::Marker steer_marker;
  steer_marker.points.reserve(2);
  steer_marker.header.frame_id = base_frame_id;
  steer_marker.type = visualization_msgs::Marker::LINE_STRIP;
  steer_marker.action = visualization_msgs::Marker::ADD;
  steer_marker.id = 3;
  steer_marker.ns = "steering cmd";
  steer_marker.color.g = 1.0;
  steer_marker.color.a = 1.0;
  steer_marker.scale.x = 0.15;
  steer_marker.scale.y = 0.1;
  steer_marker.scale.z = 0.1;
  steer_marker.pose.orientation.w = 1;
  steer_marker.pose.position.z = 0.25;
  
  point.x = STEER_MARKER_BASE[0];
  point.y = STEER_MARKER_BASE[1];
  steer_marker.points.push_back(point);
  steer_marker.points.push_back(point);

  command_markers_.markers.reserve(4);
  command_markers_.markers.emplace_back(throttle_range_marker);
  command_markers_.markers.emplace_back(steer_range_marker);
  command_markers_.markers.emplace_back(throttle_marker);
  command_markers_.markers.emplace_back(steer_marker);

  command_publisher_.publish(command_markers_);
}

void Visualizator::initFOV(const ros::NodeHandle& handle)
{
  /* Load FOV dimensioins from parameter server */
  
  float camera_x_min, camera_x_max, camera_bear_min, camera_bear_max;
  float lidar_x_min, lidar_x_max, lidar_y_min, lidar_y_max;
  std::string camera_frame_id, lidar_frame_id;

  Utils::loadParam(handle, "camera/x_min", 0.f, &camera_x_min);
  Utils::loadParam(handle, "camera/x_max", 0.f, &camera_x_max);
  Utils::loadParam(handle, "camera/bearing_min", 0.f, &camera_bear_min);
  Utils::loadParam(handle, "camera/bearing_max", 0.f, &camera_bear_max);
  Utils::loadParam(handle, "lidar/x_min", 0.f, &lidar_x_min);
  Utils::loadParam(handle, "lidar/x_max", 0.f, &lidar_x_max);
  Utils::loadParam(handle, "lidar/y_min", 0.f, &lidar_y_min);
  Utils::loadParam(handle, "lidar/y_max", 0.f, &lidar_y_max);
  Utils::loadParam(handle, "camera/frame_id", std::string("camera_center"), &camera_frame_id);
  Utils::loadParam(handle, "lidar/frame_id", std::string("lidar"), &lidar_frame_id);

  
  geometry_msgs::Point32 point;
  
  /* Init camera FOV marker parameters */
  
  camera_fov_marker_.polygon.points.reserve(4);
  camera_fov_marker_.header.frame_id = camera_frame_id;
  camera_fov_marker_.header.stamp = ros::Time();
  
  point.x = camera_x_min;
  point.y = camera_x_min * std::tan(camera_bear_max);
  camera_fov_marker_.polygon.points.push_back(point);
  
  point.x = camera_x_max;
  point.y = camera_x_max * std::tan(camera_bear_max);
  camera_fov_marker_.polygon.points.push_back(point);
  
  point.x = camera_x_max;
  point.y = camera_x_max * std::tan(camera_bear_min);
  camera_fov_marker_.polygon.points.push_back(point);
  
  point.x = camera_x_min;
  point.y = camera_x_min * std::tan(camera_bear_min);
  camera_fov_marker_.polygon.points.push_back(point);

  /* Init lidar FOV marker parameters */
  
  lidar_fov_marker_.polygon.points.reserve(4);
  lidar_fov_marker_.header.frame_id = lidar_frame_id;
  lidar_fov_marker_.header.stamp = ros::Time();

  point.x = lidar_x_min;
  point.y = lidar_y_min;
  lidar_fov_marker_.polygon.points.push_back(point);
  
  point.x = lidar_x_min;
  point.y = lidar_y_max;
  lidar_fov_marker_.polygon.points.push_back(point);
  
  point.x = lidar_x_max;
  point.y = lidar_y_max;
  lidar_fov_marker_.polygon.points.push_back(point);
  
  point.x = lidar_x_max;
  point.y = lidar_y_min;
  lidar_fov_marker_.polygon.points.push_back(point);
}

void Visualizator::cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
  static visualization_msgs::MarkerArray camera_markers;
  deleteMarkers(camera_markers, camera_publisher_);
  camera_markers.markers.reserve(msg->cones.size());
  
  int i = 0;
  for(const auto& cone : msg->cones)
  {
    if(std::isnan(cone.coords.x) || std::isnan(cone.coords.y))
      continue;
        
    camera_marker_.header = cone.coords.header;
    camera_marker_.id = i++;
    camera_marker_.pose.position.x = cone.coords.x;
    camera_marker_.pose.position.y = cone.coords.y;
    
    if(cone.color == 'b') // blue cone
    {      
      camera_marker_.color.r = 0.0;
      camera_marker_.color.g = 0.0;
      camera_marker_.color.b = 1.0;
    }
    else if(cone.color == 'y') // yellow cone
    {      
      camera_marker_.color.r = 1.0;
      camera_marker_.color.g = 1.0;
      camera_marker_.color.b = 0.0;
    }
    else if(cone.color == 's') // orange cone small
    {      
      camera_marker_.color.r = 1.0;
      camera_marker_.color.g = 0.5;
      camera_marker_.color.b = 0.0;
    }
    else if(msg->cones[i].color == 'g') // orange cone big
    {
      camera_marker_.color.r = 1.0;
      camera_marker_.color.g = 0.3;
      camera_marker_.color.b = 0.0;
    }
    else
    {
      camera_marker_.color.r = camera_marker_.color.g = camera_marker_.color.b = 0;
    }
    
    camera_markers.markers.emplace_back(camera_marker_);
  }
  
  camera_publisher_.publish(camera_markers);  
}

void Visualizator::lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
  static visualization_msgs::MarkerArray lidar_markers;
  deleteMarkers(lidar_markers, lidar_publisher_);
  lidar_markers.markers.reserve(msg->points.size());
  
  int i = 0;
  for(const auto& point : msg->points)
  {    
    lidar_marker_.header = point.header;
    lidar_marker_.id = i++;
    lidar_marker_.pose.position.x = point.x;
    lidar_marker_.pose.position.y = point.y;
    
    lidar_markers.markers.emplace_back(lidar_marker_);
  }

  lidar_publisher_.publish(lidar_markers); 
}

void Visualizator::fusionCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
  static visualization_msgs::MarkerArray fusion_markers;
  deleteMarkers(fusion_markers, fusion_publisher_);
  fusion_markers.markers.reserve(msg->cones.size());
  
  int i = 0;
  for(const auto& cone : msg->cones)
  {
    if(std::isnan(cone.coords.x) || std::isnan(cone.coords.y))
      continue;

    fusion_marker_.header = cone.coords.header;
    fusion_marker_.id = i++;
    fusion_marker_.pose.position.x = cone.coords.x;
    fusion_marker_.pose.position.y = cone.coords.y;
    
    if(cone.color == 'b') // blue cone
    {      
      fusion_marker_.color.r = 0.0;
      fusion_marker_.color.g = 0.0;
      fusion_marker_.color.b = 1.0;
    }
    else if(cone.color == 'y') // yellow cone
    {      
      fusion_marker_.color.r = 1.0;
      fusion_marker_.color.g = 1.0;
      fusion_marker_.color.b = 0.0;
    }
    else if(cone.color == 's') // orange cone small
    {      
      fusion_marker_.color.r = 1.0;
      fusion_marker_.color.g = 0.5;
      fusion_marker_.color.b = 0.0;
    }
    else if(cone.color == 'g') // orange cone big
    {
      fusion_marker_.color.r = 1.0;
      fusion_marker_.color.g = 0.3;
      fusion_marker_.color.b = 0.0;
    }
    else
    {
      fusion_marker_.color.r = fusion_marker_.color.g = fusion_marker_.color.b = 0;
    }
    fusion_markers.markers.emplace_back(fusion_marker_);
  }
  
  fusion_publisher_.publish(fusion_markers);
}

void Visualizator::poseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg)
{
  static int count = 0;
  if(!(count++ % 100)) 
  {
    geometry_msgs::Point point_car_pose;
    point_car_pose.x = msg->position.x;
    point_car_pose.y = msg->position.y;

    pose_marker_.points.emplace_back(point_car_pose);
    pose_publisher_.publish(pose_marker_);
  }

  /* publish FOV */
  camera_fov_publisher_.publish(camera_fov_marker_);
  lidar_fov_publisher_.publish(lidar_fov_marker_);
}

void Visualizator::mapCallback(const sgtdv_msgs::ConeArr::ConstPtr& msg)
{
  map_marker_.points.clear();
  map_marker_.points.reserve(msg->cones.size());
  map_marker_.colors.clear();
  map_marker_.colors.reserve(msg->cones.size());
  
  geometry_msgs::Point point_cone;
  std_msgs::ColorRGBA cone_RGBA;
  for(const auto& cone : msg->cones)
  {
    point_cone.x = cone.coords.x;
    point_cone.y = cone.coords.y;
    
    if(cone.color == 1) // blue
    {
      cone_RGBA.r = 0;  
      cone_RGBA.g = 0;
      cone_RGBA.b = 1;
    }
    else if(cone.color == 2) // yellow
    {
      cone_RGBA.r = 1;   
      cone_RGBA.g = 1;
      cone_RGBA.b = 0;
    }
    else if(cone.color == 3) // orange
    {
      cone_RGBA.r = 1;  
      cone_RGBA.g = 0.55;
      cone_RGBA.b = 0;   
    }
    cone_RGBA.a = 1;
    
    map_marker_.points.push_back(point_cone);
    map_marker_.colors.push_back(cone_RGBA);
  }
  map_publisher_.publish(map_marker_); 
}

void Visualizator::trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr& msg)
{
  trajectory_marker_.points.clear();
  trajectory_marker_.points.reserve(msg->points.size());
  
  geometry_msgs::Point trajectory_point;

  for(auto &point : msg->points)
  {
    trajectory_point.x = point.x;
    trajectory_point.y = point.y;
    trajectory_marker_.points.push_back(trajectory_point);
  }

  trajectory_publisher_.publish(trajectory_marker_);
}

void Visualizator::commandCallback(const sgtdv_msgs::Control::ConstPtr& msg)
{
  command_markers_.markers[2].points[1].x = THROTTLE_MARKER_BASE[0] + msg->speed * THROTTLE_GAIN;;
  command_markers_.markers[3].points[1].y = STEER_MARKER_BASE[1] + msg->steering_angle * STEER_GAIN;
  command_publisher_.publish(command_markers_);
}

void Visualizator::deleteMarkers(visualization_msgs::MarkerArray& marker_array,
                                const ros::Publisher& publisher) const
{
  marker_array.markers.clear();
  visualization_msgs::Marker marker;

  marker.id = 0;
  marker.action = marker.DELETEALL;
  marker_array.markers.emplace_back(marker);

  publisher.publish(marker_array);
}
