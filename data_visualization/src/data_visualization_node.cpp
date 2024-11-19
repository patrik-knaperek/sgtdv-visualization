/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/data_visualization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_visualization");
  ros::NodeHandle handle;
  DataVisualization data_visualization(handle);

  ros::spin();

  return 0;
}
