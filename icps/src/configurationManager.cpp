#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "icps/FuncSpec.h"
#include "icps/ConfSpec.h"
#include "icps/LaptopSettings.h"
#include "TLVMsg.h"
#include "ROSMsgHelper.h"

using namespace std;

#define MobSumIntervalScale	(100.0);

enum DeviceType{
	DeviceType_NotSet=0,
	DeviceType_Laptop=1,
	DeviceType_OBU=2,
	DeviceType_Smartphone=3
};

enum AppType{
	AppType_NotSet=0,
	AppType_Navigation=1
};


class ConfigurationManager {
private:
	ros::NodeHandle node;
	ros::Publisher configurationSpecPub;
	ros::Publisher laptopSettingsPub;
	ros::Subscriber funcSpecSub;

	double mobSumUpdateInterval;
	double 	speedDriftThreshold;


	vector<icps::ConfSpec*>* confSpecVec;

public:
	ConfigurationManager(){
		vector<icps::ConfSpec*> confSpecVec;
		configurationSpecPub = node.advertise<icps::ConfSpec>("icps/ConfInfo/ConfSpec", 10);
		laptopSettingsPub = node.advertise<icps::LaptopSettings>("icps/ConfInfo/LaptopSettings", 10);
		funcSpecSub = node.subscribe<icps::FuncSpec>("icps/ConfInfo/FuncSpec", 10, &ConfigurationManager::funcSpecCallback, this);

		node.param("icps/ConfParam/mobSumUpdateInterval", mobSumUpdateInterval, 1.0);
		node.param("icps/ConfParam/speedDriftThreshold", speedDriftThreshold, 0.0);
		INFMSG("Param: mobSumUpdateInterval: %f, speedDriftThreshold: %f", mobSumUpdateInterval, speedDriftThreshold);
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
			settings.speedDriftThreshold = speedDriftThreshold;
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
