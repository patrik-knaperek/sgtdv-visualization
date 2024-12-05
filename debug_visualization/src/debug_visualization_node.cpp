/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/debug_visualization.h"

constexpr int FPS = 120;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "debug_visualization");
  ros::NodeHandle handle;

  DebugVisualization debug_visualization(handle);

  ros::Rate loop_rate(FPS);
  
  while (ros::ok())
  {
    debug_visualization.publishEverythingAsArray();

    ros::spinOnce(); 
    loop_rate.sleep();
  }  

  return 0;
}
