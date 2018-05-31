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
#include <thread>
#include <mutex>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "icps/GPSSample.h"
#include "icps/MobilitySummary.h"
#include "icps/ConfSpec.h"
#include "TLVMsg.h"
#include "SocketHelper.h"
#include "ROSMsgHelper.h"

using namespace std;


//Class serves each client request
class SPInterface {
private:
	ros::Publisher sensorInfoPub;
	ros::Publisher funcSpecPub;
	ros::Subscriber confSpecSub;
	ros::Subscriber mobilitySumSub;
	ros::NodeHandle node;

	int clientSocketFD;
	long deviceID = -1;
	std::mutex sendingMutex;
	icps::ConfSpec confSpecROS;

public:
	SPInterface(int _clientSocketFD){
		clientSocketFD = _clientSocketFD;
		sensorInfoPub = node.advertise<icps::GPSSample>("SensorInfo/GSPSample", 10, true);
		funcSpecPub = node.advertise<icps::FuncSpec>("ConfInfo/FuncSpec", 10, true);
		confSpecSub = node.subscribe<icps::ConfSpec>("ConfInfo/ConfSpec", 10, &SPInterface::confSpecCallback, this);

	}

	void confSpecCallback(const icps::ConfSpec::ConstPtr& msg){
		//FIXME: This is difficult case: ConstPtr is boost shareptr const, use get() to get raw pointer

		icps::ConfSpec* receivedConfSpec = (icps::ConfSpec*) msg.get();
		confSpecROS.id = receivedConfSpec->id;
		confSpecROS.sensorSpec = receivedConfSpec->sensorSpec;
		confSpecROS.commSpec = receivedConfSpec->commSpec;
		confSpecROS.appSpec = receivedConfSpec->appSpec;
	}

	void stateWaitFuncSpec(){
		char* funcSpecTLVByte;
		do{
			ROS_INFO("Receiving funcSpec");
			funcSpecTLVByte = SocketHelper::receiveTLV(clientSocketFD);
			ROS_INFO("Received funcSpec");
		}while(!TLVHelper::isTLVType(funcSpecTLVByte, TLVMsg_FUNCTIONAL_SPEC::type));

		TLVMsg_FUNCTIONAL_SPEC funcSpec;
		funcSpec.parseTLV(funcSpecTLVByte);
		ROS_INFO("%s\n", TLVHelper::convertTLVToHexString(funcSpecTLVByte));
		ROS_INFO("%s\n", funcSpec.toString().c_str());

		deviceID = funcSpec.get_m_DEVICE_ID_V();

		icps::FuncSpec funcSpecROS;

		ROSMsgHelper::funcSpecTLVToROSMsg(&funcSpecROS, &funcSpec);

		funcSpecPub.publish(funcSpecROS);

		ros::Rate r(1);
		while(ros::ok()){
			r.sleep();
			ros::spinOnce();
			if(confSpecROS.id == deviceID){
				break;
			}

		}

		TLVMsg_CONFIGURATION_SPEC confSpec;
		ROSMsgHelper::confSpecFromMsg(&confSpec, &confSpecROS);
		char* confSpecTLVByte = confSpec.toTLV();
		ROS_INFO("%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
		ROS_INFO("%s\n", confSpec.toString().c_str());
		sendingMutex.lock();
		SocketHelper::sendTLV(clientSocketFD, confSpecTLVByte);
		sendingMutex.unlock();
	}

	void receivingThreadHandler(){
		ros::Rate r(100);
		while(!SocketHelper::isClose(clientSocketFD)){
			char* tlv = SocketHelper::receiveTLV(clientSocketFD, false);
			if(tlv != NULL){
				if(TLVHelper::isTLVType(tlv, TLVMsg_SENSOR_INFO::type)){			//SENSOR_INFO
					ROS_INFO("Receiving sensorInfo");
					TLVMsg_SENSOR_INFO sensorInfo;
					sensorInfo.parseTLV(tlv);
					ROS_INFO("%s\n", TLVHelper::convertTLVToHexString(tlv));
					ROS_INFO("%s\n", sensorInfo.toString().c_str());

					for(int i=0; i<sensorInfo.m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.size(); i++){
						icps::GPSSample gpsSampleMsg;
						ROSMsgHelper::gpsSampleTLVToROSMsg(&gpsSampleMsg, sensorInfo.get_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(i));
						sensorInfoPub.publish(gpsSampleMsg);
					}

					TLVMsg_SENSOR_INFO_ACK sensorInfoAck;
					sensorInfoAck.set_m_RESPONSE_CODE_V(1);
					char* confSpecTLVByte = sensorInfoAck.toTLV();
					ROS_INFO("%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
					ROS_INFO("%s\n", sensorInfoAck.toString().c_str());
					sendingMutex.lock();
					SocketHelper::sendTLV(clientSocketFD, confSpecTLVByte);
					sendingMutex.unlock();
				}else if(TLVHelper::isTLVType(tlv, TLVMsg_MOBILITY_SUMMARY_ACK::type)){		//MOBILITY_SUMARY_ACK
					TLVMsg_MOBILITY_SUMMARY_ACK mobSumAck;
					mobSumAck.parseTLV(tlv);
					ROS_INFO("%s\n", TLVHelper::convertTLVToHexString(tlv));
					ROS_INFO("%s\n", mobSumAck.toString().c_str());
				}
			}
			r.sleep();
		}
	}

	void mobSumCallback(const icps::MobilitySummary::ConstPtr& msg){
		ROS_INFO("Start callback function");
		TLVMsg_MOBILITY_SUMMARY mobSum;
		//FIXME: This is difficult case: ConstPtr is boost shareptr const, use get() to get raw pointer

		ROSMsgHelper::mobSumTLVFromROSMsg(&mobSum, (icps::MobilitySummary*) msg.get());
		char* mobSumTLV = mobSum.toTLV();
		ROS_INFO("%s\n", mobSum.toString().c_str());
		if(!SocketHelper::isClose(clientSocketFD)){
			sendingMutex.lock();
			SocketHelper::sendTLV(clientSocketFD, mobSumTLV);
			sendingMutex.unlock();
		}
	}


	void sendingThreadHandler(){
		mobilitySumSub = node.subscribe<icps::MobilitySummary>("MobilityInfo/MobilitySummary", 10, &SPInterface::mobSumCallback, this);
		ros::spin();
	}

	void run(){
		stateWaitFuncSpec();

		boost::thread sendingThread(&SPInterface::sendingThreadHandler, this);

		receivingThreadHandler();

		sendingThread.join();
	}
};



int gServerSocketFD;
ros::NodeHandlePtr nodePtr;
void *accept_request(void *_clientSockFDPtr)
{
	int clientSocketFD=*(int*) _clientSockFDPtr;
	free(_clientSockFDPtr);

	SPInterface cSPInterface(clientSocketFD);



	cSPInterface.run();

	int status = close(clientSocketFD);
	ROS_INFO("Finish client request");
	return NULL;
}

void signalHandler(int sig){
	close(gServerSocketFD);
	ros::shutdown();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "spInterface");
	nodePtr = boost::make_shared<ros::NodeHandle>();
	signal(SIGINT, signalHandler);

//	system("sudo killall udhcpd");
//	system("sudo wpa_cli -i wlp2s0 terminate -B");
//	sleep(1);
//	system("echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward");
//	system("sudo wpa_supplicant -d -Dnl80211 -c /etc/wpa_supplicant.conf -iwlp2s0 -B");
//	system("sudo wpa_cli -i wlp2s0 p2p_group_add");
//	system("sudo ifconfig p2p-wlp2s0-0 192.168.1.2");
//	system("sudo wpa_cli -i p2p-wlp2s0-0 p2p_find");
//	system("sudo wpa_cli -i p2p-wlp2s0-0 p2p_peers");
//	system("sudo wpa_cli -i p2p-wlp2s0-0 wps_pin any 00000000");
//	printf("\n");
//	system("sudo udhcpd /etc/udhcpd.conf &");

	SocketHelper::createServerSockAndWaitForConnection("192.168.1.2", 6278, false, &gServerSocketFD, &accept_request);
	return 0;
}
