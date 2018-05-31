#include "ros/ros.h"
#include "TLVMsg.h"
#include "SocketHelper.h"
#include "icps/VehicleInfo.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "dummyLaptopForLidar");
	ros::NodeHandle node;

	long dimension = 0;
	TLVHelper::appendIntToLong(&dimension, 10, 4, 4);
	TLVHelper::appendIntToLong(&dimension, 20, 0, 4);

	INFMSG("%ld\n", dimension);

	return 0;
}


