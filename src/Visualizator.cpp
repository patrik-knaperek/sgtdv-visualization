/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/Visualizator.h"

Visualizator::Visualizator(ros::NodeHandle& handle)
{
	/* ROS interface init */
	camera_publisher_ = handle.advertise<visualization_msgs::MarkerArray>("camera_cones/marker",1);
	lidar_publisher_ = handle.advertise<visualization_msgs::MarkerArray>("lidar_cones/marker",1);
	fusion_publisher_ = handle.advertise<visualization_msgs::MarkerArray>("fusion_cones/marker",1);
	pose_publisher_ = handle.advertise<visualization_msgs::Marker>("slam/pose/marker", 1);
	map_publisher_ = handle.advertise<visualization_msgs::Marker>("slam/map/marker", 1);
	trajectory_publisher_ = handle.advertise<visualization_msgs::Marker>("pathplanning_trajectory/marker",1);
	command_publisher_ = handle.advertise<visualization_msgs::MarkerArray>("pathtracking_commands/marker", 1);
	
	camera_subscriber_ = handle.subscribe("camera_cones", 1, &Visualizator::cameraCallback, this);
	lidar_subscriber_ = handle.subscribe("lidar_cones", 1, &Visualizator::lidarCallback, this);
	fusion_subscriber_ = handle.subscribe("fusion_cones", 1, &Visualizator::fusionCallback, this);
	pose_subscriber_ = handle.subscribe("slam/pose", 1, &Visualizator::poseCallback, this);
	map_subscriber_ = handle.subscribe("slam/map", 1, &Visualizator::mapCallback, this);
	trajectory_subscriber_ = handle.subscribe("pathplanning_trajectory", 1, &Visualizator::trajectoryCallback, this);
	command_subscriber_ = handle.subscribe("pathtracking_commands", 1, &Visualizator::commandCallback, this);

	initCommandMarkers(handle);
}

void Visualizator::initCommandMarkers(const ros::NodeHandle& handle)
{
	double steer_min, steer_max;
	int throtle_min, throtle_max;
	std::string base_frame_id;
	Utils::loadParam(handle, "/controller/steering/min", -1.0, &steer_min);
	Utils::loadParam(handle, "/controller/steering/max", 1.0, &steer_max);
	Utils::loadParam(handle, "/controller/speed/min", 0, &throtle_min);
	Utils::loadParam(handle, "/controller/speed/max", 0, &throtle_max);
	Utils::loadParam(handle, "/base_frame_id", std::string("base_link"), &base_frame_id);

	geometry_msgs::Point point;
	

	visualization_msgs::Marker throtle_range_marker;
	throtle_range_marker.header.frame_id = base_frame_id;
	throtle_range_marker.type = visualization_msgs::Marker::LINE_STRIP;
	throtle_range_marker.action = visualization_msgs::Marker::ADD;
	throtle_range_marker.id = 0;
	throtle_range_marker.ns = "throtle range";
	throtle_range_marker.color.r = 1.0;
	throtle_range_marker.color.a = 0.2;
	throtle_range_marker.scale.x = 0.15;
	throtle_range_marker.scale.y = 0.1;
	throtle_range_marker.scale.z = 0.1;
	throtle_range_marker.pose.orientation.w = 1;
	throtle_range_marker.pose.position.z = 0.2;
	
	point.x = THROTLE_MARKER_BASE[0] + throtle_min * THROTTLE_GAIN;
	point.y = THROTLE_MARKER_BASE[1];
	throtle_range_marker.points.push_back(point);
	point.x = THROTLE_MARKER_BASE[0] + throtle_max * THROTTLE_GAIN;
	throtle_range_marker.points.push_back(point);
	
	visualization_msgs::Marker steer_range_marker;
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

	visualization_msgs::Marker throtle_marker;
	throtle_marker.header.frame_id = base_frame_id;
	throtle_marker.type = visualization_msgs::Marker::LINE_STRIP;
	throtle_marker.action = visualization_msgs::Marker::ADD;
	throtle_marker.id = 2;
	throtle_marker.ns = "throtle cmd";
	throtle_marker.color.g = 1.0;
	throtle_marker.color.a = 1.0;
	throtle_marker.scale.x = 0.15;
	throtle_marker.scale.y = 0.1;
	throtle_marker.scale.z = 0.1;
	throtle_marker.pose.orientation.w = 1;
	throtle_marker.pose.position.z = 0.25;
	
	point.x = THROTLE_MARKER_BASE[0];
	point.y = THROTLE_MARKER_BASE[1];
	throtle_marker.points.push_back(point);
	throtle_marker.points.push_back(point);

	visualization_msgs::Marker steer_marker;
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

	command_marker_.markers.push_back(throtle_range_marker);
	command_marker_.markers.push_back(steer_range_marker);
	command_marker_.markers.push_back(throtle_marker);
	command_marker_.markers.push_back(steer_marker);

	command_publisher_.publish(command_marker_);
}

void Visualizator::cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
	static visualization_msgs::MarkerArray cameraMarkers;
	deleteMarkers(cameraMarkers, camera_publisher_);
	cameraMarkers.markers.reserve(msg->cones.size());
	
	int i = 0;
	for(const auto& cone : msg->cones)
	{
		if (std::isnan(cone.coords.x) || std::isnan(cone.coords.y))
			continue;
		
		visualization_msgs::Marker marker;
		
		marker.header = cone.coords.header;
		marker.type = marker.SPHERE;
		marker.action = marker.ADD;
		marker.id = i++;
		marker.lifetime = ros::Duration(0.5);
		marker.pose.position.x = cone.coords.x;
		marker.pose.position.y = cone.coords.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
	   
		marker.color.a = 0.6;
		
		if (cone.color == 'b') // blue cone
		{		  
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		}
		else if (cone.color == 'y') // yellow cone
		{		  
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		else if (cone.color == 's') // orange cone small
		{		  
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
		}
		else if (msg->cones[i].color == 'g') // orange cone big
		{
			marker.color.r = 1.0;
			marker.color.g = 0.3;
			marker.color.b = 0.0;
		}
		else
		{
			marker.color.r = marker.color.g = marker.color.b = 0;
		}
		
		cameraMarkers.markers.push_back(marker);
	}
	
	camera_publisher_.publish(cameraMarkers);  
}

void Visualizator::lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
	static visualization_msgs::MarkerArray lidarMarkers;
	deleteMarkers(lidarMarkers, lidar_publisher_);
	lidarMarkers.markers.reserve(msg->points.size());
	visualization_msgs::Marker marker;
	
	int i = 0;
	for (const auto& point : msg->points)
	{
		marker.header = point.header;
		marker.type = marker.CYLINDER;
		marker.action = marker.ADD;
		marker.id = i++;
		marker.lifetime = ros::Duration(0.5);
		marker.pose.position.x = point.x;
		marker.pose.position.y = point.y;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 0.7;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		
		lidarMarkers.markers.push_back(marker);
	}

	lidar_publisher_.publish(lidarMarkers); 
}

void Visualizator::fusionCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
	static visualization_msgs::MarkerArray fusionMarkers;
	deleteMarkers(fusionMarkers, fusion_publisher_);
	fusionMarkers.markers.reserve(msg->cones.size());
	
	visualization_msgs::Marker marker;
	
	
	int i = 0;
	for(const auto& cone : msg->cones)
	{
		if (std::isnan(cone.coords.x) || std::isnan(cone.coords.y))
			continue;

		marker.header = cone.coords.header;
		marker.type = marker.CUBE;
		marker.action = marker.ADD;
		marker.id = i++;
		marker.lifetime = ros::Duration(0.5);
		marker.pose.position.x = cone.coords.x;
		marker.pose.position.y = cone.coords.y;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
	
		marker.color.a = 1.0;
		if (cone.color == 'b') // blue cone
		{		  
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		}
		else if (cone.color == 'y') // yellow cone
		{		  
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		else if (cone.color == 's') // orange cone small
		{		  
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
		}
		else if (cone.color == 'g') // orange cone big
		{
			marker.color.r = 1.0;
			marker.color.g = 0.3;
			marker.color.b = 0.0;
		}
		else
		{
			marker.color.r = marker.color.g = marker.color.b = 0;
		}
		fusionMarkers.markers.push_back(marker);
	}
	
	fusion_publisher_.publish(fusionMarkers);
}

void Visualizator::poseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg)
{
	static visualization_msgs::Marker carPoseMarker;
	geometry_msgs::Point pointCarPose;
	static int count = 0;
	if (!(count++ % 100)) 
	{
		carPoseMarker.header.frame_id =  "map";
		carPoseMarker.header.stamp = ros::Time();
		carPoseMarker.type = visualization_msgs::Marker::POINTS;
		carPoseMarker.action = visualization_msgs::Marker::ADD;
		carPoseMarker.id = 0;
		carPoseMarker.lifetime = ros::Duration(0);
		carPoseMarker.scale.x = 0.3;
		carPoseMarker.scale.y = 0.3;
		carPoseMarker.color.a = 1;
		carPoseMarker.color.r = 1;

		pointCarPose.x = msg->position.x;
		pointCarPose.y = msg->position.y;

		carPoseMarker.points.push_back(pointCarPose);
		pose_publisher_.publish(carPoseMarker);
	}
}

void Visualizator::mapCallback(const sgtdv_msgs::ConeArr::ConstPtr& msg)
{
	static sgtdv_msgs::ConeArr coneArr;
	coneArr.cones.clear();

	static visualization_msgs::Marker coneMarker;
	coneMarker.points.clear();
	coneMarker.colors.clear();
	
	coneMarker.header.frame_id = "map";
	coneMarker.header.stamp = ros::Time::now();
	coneMarker.type = visualization_msgs::Marker::POINTS;
	coneMarker.action = visualization_msgs::Marker::ADD;
	
	geometry_msgs::Point pointCone;
	std_msgs::ColorRGBA coneRGBA;
	for (const auto& cone : msg->cones)
	{
		coneMarker.scale.x = 0.2;
		coneMarker.scale.y = 0.2;
		
		pointCone.x = cone.coords.x;
		pointCone.y = cone.coords.y;
		
		if(cone.color == 1) // blue
		{
			coneRGBA.r = 0;	
			coneRGBA.g = 0;
			coneRGBA.b = 1;
		}
		else if(cone.color == 2) // yellow
		{
			coneRGBA.r = 1;   
			coneRGBA.g = 1;
			coneRGBA.b = 0;
		}
		else if(cone.color == 3) // orange
		{
			coneRGBA.r = 1;	
			coneRGBA.g = 0.55;
			coneRGBA.b = 0;   
		}
		coneRGBA.a = 1;
		
		coneMarker.points.push_back(pointCone);
		coneMarker.colors.push_back(coneRGBA);
	}
	map_publisher_.publish(coneMarker); 
}

void Visualizator::trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr& msg)
{
	static visualization_msgs::Marker trajectory_marker;
	trajectory_marker.points.clear();
	trajectory_marker.points.reserve(msg->points.size());

	trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
	trajectory_marker.header.frame_id = "map";
	trajectory_marker.scale.x = 0.2;
	trajectory_marker.scale.y = 0.2;
	trajectory_marker.color.r = 1.0f;
	trajectory_marker.color.a = 1.0;
	trajectory_marker.pose.orientation.w = 1.0;
	
	geometry_msgs::Point trajectory_point;

	for (auto &point : msg->points)
	{
		trajectory_point.x = point.x;
		trajectory_point.y = point.y;
		trajectory_marker.points.push_back(trajectory_point);
	}

	trajectory_publisher_.publish(trajectory_marker);
}

void Visualizator::commandCallback(const sgtdv_msgs::Control::ConstPtr& msg)
{
	command_marker_.markers[2].points[1].x = THROTLE_MARKER_BASE[0] + msg->speed * THROTTLE_GAIN;;
	command_marker_.markers[3].points[1].y = STEER_MARKER_BASE[1] + msg->steeringAngle * STEER_GAIN;
	command_publisher_.publish(command_marker_);
}

void Visualizator::deleteMarkers(visualization_msgs::MarkerArray& marker_array,
								const ros::Publisher& publisher) const
{
	marker_array.markers.clear();
	visualization_msgs::Marker marker;

	marker.id = 0;
	marker.action = marker.DELETEALL;
	marker_array.markers.push_back(marker);

	publisher.publish(marker_array);
}

						
