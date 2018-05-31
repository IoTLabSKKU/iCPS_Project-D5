#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <execinfo.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <unordered_map>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "icps/MobilitySummary.h"
#include "icps/VehicleInfo.h"
#include "icps/ObjectList.h"
#include "icps/LaptopSettings.h"
#include "TLVMsg.h"
#include "SocketHelper.h"
#include "ROSMsgHelper.h"

using namespace std;
ros::Publisher mobilityInfoPub;

#define rvizScale	(100000)
#define lonOffset	(129.083)
#define latOffset	(35.233)

enum MarkerType{
	SelfMarker,
	NeighborMarker,
	PedestrianMarker,
	RSUMarker,
	ObjectMarker
};

class MobilityManager {
private:
	ros::Publisher mobSumPub;
	ros::Publisher selfMobilityPub;
	ros::Subscriber selfGPSSampleSub;
	ros::Subscriber neighborGPSSampleSub;
	ros::Subscriber pedestrianGPSSampleSub;
	ros::Subscriber rsuGPSSampleSub;
	ros::Subscriber objectListSub;
	ros::Subscriber laptopSettingsSub;
	ros::NodeHandle node;

	ros::Publisher		rvizSelfMarkerPub;
	ros::Publisher		rvizNeighborMarkerPub;
	ros::Publisher		rvizPedestrianMarkerPub;
	ros::Publisher		rvizRSUMarkerPub;
	ros::Publisher		rvizObjectMarkerPub;

	tf::TransformBroadcaster br;
	tf::Transform transform;

	double mobSumUpdateInterval;
	double remainingTimeToSendMobSum;
	double speedDriftThreshold;

	bool hasPubMobSum;
	bool hasPubVehInfo;

	long deviceID = -1;

	vector<icps::GPSSample> selfGPSSampleList;
	unordered_map<unsigned long, icps::NeighborInfo> neighborGPSSampleList;
	unordered_map<unsigned long, icps::PedestrianInfo> pedestrianGPSSampleList;
	unordered_map<unsigned long, icps::RSUInfo> rsuGPSSampleList;
	icps::ObjectList objectList;

	const long timeGapToSearchForSelfGPS = 200;
public:
	MobilityManager(){
		mobSumPub = node.advertise<icps::MobilitySummary>("icps/MobilityInfo/MobilitySummary", 10);
		selfMobilityPub = node.advertise<icps::VehicleInfo>("icps/MobilityInfo/SelfMobility", 10);

		laptopSettingsSub = node.subscribe<icps::LaptopSettings>("icps/ConfInfo/LaptopSettings", 10, &MobilityManager::laptopSettingsCallback, this);
		selfGPSSampleSub = node.subscribe<icps::GPSSample>("icps/SensorInfo/SelfGPSSample", 10, &MobilityManager::selfGPSSampleCallback, this);
		neighborGPSSampleSub = node.subscribe<icps::NeighborInfo>("icps/SensorInfo/NeighborGPSSample", 10, &MobilityManager::neighborGPSSampleCallback, this);
		pedestrianGPSSampleSub = node.subscribe<icps::PedestrianInfo>("icps/SensorInfo/PedestrianGPSSample", 10, &MobilityManager::pedestrianGPSSampleCallback, this);
		rsuGPSSampleSub = node.subscribe<icps::RSUInfo>("icps/SensorInfo/RSUGPSSample", 10, &MobilityManager::rsuGPSSampleCallback, this);
		objectListSub = node.subscribe<icps::ObjectList>("icps/SensorInfo/ObjectList", 10, &MobilityManager::objectListCallback, this);

		rvizSelfMarkerPub = node.advertise<visualization_msgs::Marker>("icps/rviz/SelfMarker", 10);
		rvizNeighborMarkerPub = node.advertise<visualization_msgs::Marker>("icps/rviz/NeighborMarker", 10);
		rvizPedestrianMarkerPub = node.advertise<visualization_msgs::Marker>("icps/rviz/PedestrianMarker", 10);
		rvizRSUMarkerPub = node.advertise<visualization_msgs::Marker>("icps/rviz/RSUMarker", 10);
		rvizObjectMarkerPub = node.advertise<visualization_msgs::Marker>("icps/rviz/ObjectMarker", 10);

		hasPubMobSum = true;
		hasPubVehInfo = true;

		mobSumUpdateInterval = 1;
		remainingTimeToSendMobSum = 0;
	}

	visualization_msgs::Marker createMarker(MarkerType _type, long _id, double _posX, double _posY, double _orienX, double _orienY, double _scaleX=1, double _scaleY=1){
		switch(_type){
		case SelfMarker:
			return ROSHelper::createMarker("map", "SelfVehicle", _id, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, _posX, _posY, _orienX, _orienY, _scaleX, _scaleY, 1, 0, 0, 10000);
			break;
		case NeighborMarker:
			return ROSHelper::createMarker("map", "Neighbor", _id, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, _posX, _posY, _orienX, _orienY, _scaleX, _scaleY, 0, 1, 0, 10000);
			break;
		case PedestrianMarker:
			return ROSHelper::createMarker("map", "Pedestrian", _id, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, _posX, _posY, _orienX, _orienY, _scaleX, _scaleY, 0, 0, 1, 10000);
			break;
		case RSUMarker:
			return ROSHelper::createMarker("map", "RSU", _id, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, _posX, _posY, _orienX, _orienY, _scaleX, _scaleY, 1, 0, 1, 10000);
			break;
		case ObjectMarker:
			return ROSHelper::createMarker("map", "Object", _id, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, _posX, _posY, _orienX, _orienY, _scaleX, _scaleY, 1, 1, 0, 1);
			break;
		}
	}

	void laptopSettingsCallback(const icps::LaptopSettings::ConstPtr& msg){
		icps::LaptopSettings* laptopSettings = (icps::LaptopSettings*) msg.get();
		mobSumUpdateInterval = laptopSettings->mobSumUpdateInterval;
		speedDriftThreshold = laptopSettings->speedDriftThreshold;
	}

	void selfGPSSampleCallback(const icps::GPSSample::ConstPtr& msg){
		icps::GPSSample* gpsSample = (icps::GPSSample*) msg.get();

		if(gpsSample->speed/SpeedScale < speedDriftThreshold){
			return;
		}

		selfGPSSampleList.push_back(*gpsSample);
		hasPubMobSum = false;
		hasPubVehInfo = false;

		transform.setOrigin(tf::Vector3((gpsSample->longitude/10000000.0 - lonOffset) * rvizScale, (gpsSample->latitude/10000000.0 - latOffset) * rvizScale, 0.0) );
		transform.setRotation(tf::Quaternion(0, 0, gpsSample->heading / HeadingScale, 1) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "selfVehicle"));
		rvizSelfMarkerPub.publish(createMarker(SelfMarker, 0, (gpsSample->longitude/10000000.0 - lonOffset) * rvizScale, (gpsSample->latitude/10000000.0 - latOffset) * rvizScale, 0, 0));
	}

	void neighborGPSSampleCallback(const icps::NeighborInfo::ConstPtr& msg){
		icps::NeighborInfo* info = (icps::NeighborInfo*) msg.get();
		auto searchIterator = neighborGPSSampleList.find(info->vehicleID);
		if(searchIterator != neighborGPSSampleList.end()){
			neighborGPSSampleList.erase(searchIterator);
		}
		neighborGPSSampleList.insert({info->vehicleID,*info});
		hasPubMobSum = false;

		rvizNeighborMarkerPub.publish(createMarker(NeighborMarker, info->vehicleID, (info->gpsSample.longitude/10000000.0 - lonOffset) * rvizScale, (info->gpsSample.latitude/10000000.0 - latOffset) * rvizScale, 0, 0));
	}

	void pedestrianGPSSampleCallback(const icps::PedestrianInfo::ConstPtr& msg){
		icps::PedestrianInfo* info = (icps::PedestrianInfo*) msg.get();
		auto searchIterator = pedestrianGPSSampleList.find(info->pedestrianID);
		if(searchIterator != pedestrianGPSSampleList.end()){
			pedestrianGPSSampleList.erase(searchIterator);
		}
		pedestrianGPSSampleList.insert({info->pedestrianID,*info});
		hasPubMobSum = false;

		rvizPedestrianMarkerPub.publish(createMarker(PedestrianMarker, info->pedestrianID, (info->gpsSample.longitude/10000000.0 - lonOffset) * rvizScale, (info->gpsSample.latitude/10000000.0 - latOffset) * rvizScale, 0, 0));
	}

	void rsuGPSSampleCallback(const icps::RSUInfo::ConstPtr& msg){
		icps::RSUInfo* info = (icps::RSUInfo*) msg.get();
		auto searchIterator = rsuGPSSampleList.find(info->RSUID);
		if(searchIterator != rsuGPSSampleList.end()){
			rsuGPSSampleList.erase(searchIterator);
		}
		rsuGPSSampleList.insert({info->RSUID,*info});
		hasPubMobSum = false;

		rvizRSUMarkerPub.publish(createMarker(RSUMarker, info->RSUID, (info->longitude/10000000.0 - lonOffset) * rvizScale, (info->latitude/10000000.0 - latOffset) * rvizScale, 0, 0));
	}

	void objectListCallback(const icps::ObjectList::ConstPtr& msg){
		objectList = *((icps::ObjectList*) msg.get());
		hasPubMobSum = false;

		for(auto cObject : objectList.objectList){
			rvizObjectMarkerPub.publish(createMarker(ObjectMarker, cObject.objectID, (cObject.gpsSample.longitude/10000000.0 - lonOffset) * rvizScale, (cObject.gpsSample.latitude/10000000.0 - latOffset) * rvizScale, 0, 0, cObject.width, cObject.length));
		}
	}

	/**
	 * Get the highest accuracy sample within 0.2 second
	 */
	void mobSumPubFunction(){
		if(selfGPSSampleList.size() > 0){
			if(hasPubMobSum == false){
				icps::MobilitySummary pubMsg;
				pubMsg.selfMobility = selfGPSSampleList.back();
				for(auto kv : neighborGPSSampleList){
					pubMsg.neighborList.push_back(kv.second);
				}
				for(auto kv : pedestrianGPSSampleList){
					pubMsg.pedestrianList.push_back(kv.second);
				}
				for(auto kv : rsuGPSSampleList){
					pubMsg.rsuList.push_back(kv.second);
				}
				pubMsg.objectList = objectList.objectList;
				mobSumPub.publish(pubMsg);
				hasPubMobSum = true;
			}
		}
	}

	void selfVehInfoPubFunction(){
		if(selfGPSSampleList.size() > 0){
			if(hasPubVehInfo == false){
				icps::VehicleInfo pubMsg;
				pubMsg.gpsSample = selfGPSSampleList.back();
				selfMobilityPub.publish(pubMsg);
				hasPubVehInfo = true;
			}
		}
	}

	void run(){
		ros::Rate r(10);
		while(ros::ok()){
			r.sleep();
			ros::spinOnce();
			//TODO: fix pub time of two functions
			if(remainingTimeToSendMobSum < 0){
				mobSumPubFunction();
				remainingTimeToSendMobSum = mobSumUpdateInterval;
			}else{
				remainingTimeToSendMobSum -= 1/10;
			}
			selfVehInfoPubFunction();
		}
	}
};


int main(int argc, char **argv){
	ros::init(argc, argv, "mobilityManager");
	ros::NodeHandle n;

	MobilityManager mobilityManager;

	mobilityManager.run();

	return 0;
}
