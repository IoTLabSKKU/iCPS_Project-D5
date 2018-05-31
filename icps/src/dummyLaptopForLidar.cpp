#include "ros/ros.h"
#include "TLVMsg.h"
#include "SocketHelper.h"
#include "icps/VehicleInfo.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "dummyLaptopForLidar");
	ros::NodeHandle node;

	ros::Publisher selfMobilityPub = node.advertise<icps::VehicleInfo>("icps/MobilityInfo/SelfMobility", 10);

	int i = 0;
	ros::Rate r(1);
	while(ros::ok()){
		r.sleep();
		ros::spinOnce();
		icps::VehicleInfo pubMsg;
		pubMsg.gpsSample.utcTime = 1520469733221;
		pubMsg.gpsSample.latitude = 352333090 + i*10;
		pubMsg.gpsSample.longitude = 1290830950;
		pubMsg.gpsSample.heading = 0;
		pubMsg.gpsSample.speed = 0;
		selfMobilityPub.publish(pubMsg);
		i++;
	}

	return 0;
}


