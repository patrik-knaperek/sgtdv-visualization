/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/Visualizator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizator");
	ros::NodeHandle handle;
	Visualizator visualizatorObj(handle);

	ros::spin();

	return 0;
}
