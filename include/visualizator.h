/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>

/* SGT */
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include <sgtdv_msgs/FusionMsg.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/Control.h>
#include "../../SGT_Utils.h"

class Visualizator
{
	public:
		Visualizator(ros::NodeHandle& handle);
		~Visualizator() = default;
		
		void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg) const;
		void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg) const;
		void fusionCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg) const;
		
		void poseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg) const;
		void mapCallback(const sgtdv_msgs::ConeArr::ConstPtr& msg) const;
		void trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr& msg) const;
		void commandCallback(const sgtdv_msgs::Control::ConstPtr& msg);

	public:
		static constexpr double THROTTLE_MARKER_BASE[2] = {-1.0, 1.5};
		static constexpr double THROTTLE_GAIN = 1 / 50.0;
		static constexpr double STEER_MARKER_BASE[2] = {2.0, 0.0};
		static constexpr double STEER_GAIN = 2;
	
	private:
		void initCommandMarkers(const ros::NodeHandle& handle);
		void initFOV(const ros::NodeHandle& handle);		
		void deleteMarkers(visualization_msgs::MarkerArray& marker_array,
						const ros::Publisher& publisher) const;

		ros::Publisher camera_publisher_;
		ros::Publisher lidar_publisher_;
		ros::Publisher fusion_publisher_;
		ros::Publisher pose_publisher_;
		ros::Publisher map_publisher_;
		ros::Publisher trajectory_publisher_;
		ros::Publisher command_publisher_;
		ros::Publisher camera_fov_publisher_;
		ros::Publisher lidar_fov_publisher_;

		ros::Subscriber camera_subscriber_;
		ros::Subscriber lidar_subscriber_;
		ros::Subscriber fusion_subscriber_;
		ros::Subscriber pose_subscriber_;
		ros::Subscriber map_subscriber_;
		ros::Subscriber trajectory_subscriber_;
		ros::Subscriber command_subscriber_;

		visualization_msgs::Marker steering_marker_, throtle_marker_;
		visualization_msgs::MarkerArray command_markers_;
		geometry_msgs::PolygonStamped camera_fov_marker_, lidar_fov_marker_;
};
