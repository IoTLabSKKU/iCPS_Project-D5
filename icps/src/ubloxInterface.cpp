#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "ROSMsgHelper.h"

using namespace std;

class UbloxInterface {
private:
	ros::NodeHandle node;
	ros::Publisher sensorInfoPub;
	ros::Publisher ubloxStatusPub;

	ros::Subscriber fixSub;
	ros::Subscriber fixVelocitySub;
	ros::Subscriber navpvtSub;

public:
	UbloxInterface(){
		sensorInfoPub = node.advertise<icps::GPSSample>("icps/SensorInfo/SensorInfo", 10);

		funcSpecSub = node.subscribe<icps::FuncSpec>("icps/ConfInfo/FuncSpec", 10, &ConfigurationManager::funcSpecCallback, this);

		node.param("icps/ConfParam/mobSumUpdateInterval", mobSumUpdateInterval, 1.0);
		node.param("icps/ConfParam/enableGPSDrifting", enableGPSDrifting, 0);
		INFMSG("Param: mobSumUpdateInterval: %f, enableGPSDrifting: %d", mobSumUpdateInterval, enableGPSDrifting);
	}

	void funcSpecCallback(const icps::FuncSpec::ConstPtr& msg){
		INFMSG("Start callback function");

		icps::FuncSpec* funcSpec = (icps::FuncSpec*) msg.get();
		icps::ConfSpec* confSpec = new icps::ConfSpec();
		confSpec->id = funcSpec->id;
		confSpec->sensorSpec = funcSpec->sensorSpec;
		confSpec->commSpec = funcSpec->commSpec;
		confSpec->appSpec = funcSpec->appSpec;
		for(int i=0; i<confSpec->sensorSpec.size(); i++){
			if(funcSpec->type == DeviceType_OBU){
				confSpec->sensorSpec.at(i).inUse = 1;
			}else{
				confSpec->sensorSpec.at(i).inUse = 0;
			}
		}
		for(int i=0; i<confSpec->commSpec.size(); i++){
			confSpec->commSpec.at(i).inUse = 1;
		}
		for(int i=0; i<confSpec->appSpec.size(); i++){
			confSpec->appSpec.at(i).inUse = 1;
			if(funcSpec->type == DeviceType_Smartphone && funcSpec->appSpec.at(i).type == AppType_Navigation){
				mobSumUpdateInterval = TLVHelper::getIntFromLong(funcSpec->appSpec.at(i).spec, 4, 2, false) / MobSumIntervalScale;
			}
		}
	//	confSpecVec->push_back(confSpec);
		configurationSpecPub.publish(*confSpec);
		//FIXME: do actual computation here rather than enabling everything
	}

	void run(){
		ros::Rate r(1);
		while(ros::ok()){
			r.sleep();
			ros::spinOnce();
			icps::LaptopSettings settings;
			settings.mobSumUpdateInterval = mobSumUpdateInterval;
			settings.enableGPSDrifting = enableGPSDrifting;
			laptopSettingsPub.publish(settings);
		}

		//TODO: Wait for 5 second at the beginning and compute the optimal configuration, after that if receiving new functional spec then reconfigurate
	}
};


int main(int argc, char **argv){
	ros::init(argc, argv, "configurationInfoManager");
	ros::NodeHandle n;
	string configFileName;

	ConfigurationManager configurationManager;

	configurationManager.run();

	return 0;
}
