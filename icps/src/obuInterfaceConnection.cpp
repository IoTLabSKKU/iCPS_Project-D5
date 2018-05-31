/*
 * obuInterfaceConnection.cpp
 *
 *  Created on: Jul 1, 2016
 *      Author: retry
 */

#include "obuInterfaceConnection.h"


void obuInterfaceConnection::start(){
	INFMSG("Connection start\n");

	selfSensorInfoPub = node.advertise<icps::GPSSample>("icps/SensorInfo/SelfGPSSample", 10);
	neighborSensorInfoPub = node.advertise<icps::NeighborInfo>("icps/SensorInfo/NeighborGPSSample", 10);
	pedestrianSensorInfoPub = node.advertise<icps::PedestrianInfo>("icps/SensorInfo/PedestrianGPSSample", 10);
	rsuSensorInfoPub = node.advertise<icps::RSUInfo>("icps/SensorInfo/RSUGPSSample", 10);
	funcSpecPub = node.advertise<icps::FuncSpec>("icps/ConfInfo/FuncSpec", 10);
	confSpecSub = node.subscribe<icps::ConfSpec>("icps/ConfInfo/ConfSpec", 10, &obuInterfaceConnection::confSpecCallback, this);
	selfMobilitySub = node.subscribe<icps::VehicleInfo>("icps/MobilityInfo/SelfMobility", 10, &obuInterfaceConnection::selfMobCallback, this);
	mState = waitFuncSpec;
	ros::Rate(1).sleep();

	receiveTLV();
}

void obuInterfaceConnection::receiveTLVCallback(std::vector<char> _tlv){
	INFMSG("ReceiveTLVCallback\n");

	char* tlv = _tlv.data();
	switch(mState){
	case waitFuncSpec:
		if(TLVHelper::isTLVType(tlv, TLVMsg_FUNCTIONAL_SPEC::type)){
			INFMSG("receiveTLVCallback funcSpec\n");

			TLVMsg_FUNCTIONAL_SPEC funcSpec;
			funcSpec.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", funcSpec.toString().c_str());

			mDeviceID = funcSpec.get_m_DEVICE_ID_V();

			icps::FuncSpec funcSpecROS;
			ROSMsgHelper::funcSpecTLVToROSMsg(&funcSpecROS, &funcSpec);

			funcSpecPub.publish(funcSpecROS);

			while(ros::ok()){
				ros::Rate(10).sleep();
				if(confSpecROS.id == mDeviceID){
					break;
				}
			}

			TLVMsg_CONFIGURATION_SPEC confSpec;
			ROSMsgHelper::confSpecFromMsg(&confSpec, &confSpecROS);
			char* confSpecTLVByte = confSpec.toTLV();
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
			INFMSG("%s\n", confSpec.toString().c_str());

			sendTLV(confSpecTLVByte);

			mState = waitSensorInfo;

			//Receive in the new thread
			receiveTLV();
		}else{
			ERRMSG("Expect FuncSpecTLV but receive: %s\n", TLVHelper::convertTLVToHexString(tlv));
			receiveTLV();
		}
		break;

	case waitSensorInfo:
		if(TLVHelper::isTLVType(tlv, TLVMsg_SENSOR_INFO::type)){			//SENSOR_INFO
			INFMSG("receiveTLVCallback sensorInfo\n");

			TLVMsg_SENSOR_INFO sensorInfo;
			sensorInfo.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", sensorInfo.toString().c_str());

			for(int i=0; i<sensorInfo.m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.size(); i++){
				icps::GPSSample gpsSampleMsg;
				ROSMsgHelper::gpsSampleTLVToROSMsg(&gpsSampleMsg, sensorInfo.get_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(i));
				selfSensorInfoPub.publish(gpsSampleMsg);
			}

			TLVMsg_SENSOR_INFO_ACK sensorInfoAck;
			sensorInfoAck.set_m_RESPONSE_CODE_V(1);
			char* confSpecTLVByte = sensorInfoAck.toTLV();
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
			INFMSG("%s\n", sensorInfoAck.toString().c_str());
			sendTLV(confSpecTLVByte);
			receiveTLV();
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_REMOTE_INFO::type)){			//REMOTE_INFO
			INFMSG("receiveTLVCallback remoteInfo\n");

			TLVMsg_REMOTE_INFO tlvObject;
			tlvObject.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", tlvObject.toString().c_str());

			for(int i=0; i<tlvObject.m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.size(); i++){
				icps::NeighborInfo msg;
				msg.vehicleID = tlvObject.get_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(i);
				ROSMsgHelper::gpsSampleTLVToROSMsg(&msg.gpsSample, tlvObject.get_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(i));
				neighborSensorInfoPub.publish(msg);
			}

			for(int i=0; i<tlvObject.m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.size(); i++){
				icps::PedestrianInfo msg;
				msg.pedestrianID = tlvObject.get_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(i);
				ROSMsgHelper::gpsSampleTLVToROSMsg(&msg.gpsSample, tlvObject.get_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(i));
				pedestrianSensorInfoPub.publish(msg);
			}

			for(int i=0; i<tlvObject.m_RSU_LIST.m_RSU_INFO_vec.size(); i++){
				icps::RSUInfo msg;
				ROSMsgHelper::rsuInfoTLVToROSMsg(&msg, tlvObject.get_m_RSU_LIST_m_RSU_INFO_vec_V(i));
				rsuSensorInfoPub.publish(msg);
			}

			TLVMsg_REMOTE_INFO_ACK tlvAckObject;
			tlvAckObject.set_m_RESPONSE_CODE_V(1);
			char* confSpecTLVByte = tlvAckObject.toTLV();
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
			INFMSG("%s\n", tlvAckObject.toString().c_str());
			sendTLV(confSpecTLVByte);
			receiveTLV();
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_MOBILITY_SUMMARY_ACK::type)){		//MOBILITY_SUMARY_ACK
			INFMSG("receiveTLVCallback mobSumAck\n");

			TLVMsg_MOBILITY_SUMMARY_ACK mobSumAck;
			mobSumAck.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", mobSumAck.toString().c_str());
			receiveTLV();
		}
	}
}

void obuInterfaceConnection::laptopSettingsCallback(const icps::LaptopSettings::ConstPtr& msg){
	icps::LaptopSettings* laptopSettings = (icps::LaptopSettings*) msg.get();
}

void obuInterfaceConnection::confSpecCallback(const icps::ConfSpec::ConstPtr& msg){
	INFMSG("confSpecCallback\n");

	icps::ConfSpec* receivedConfSpec = (icps::ConfSpec*) msg.get();
	if(receivedConfSpec->id == mDeviceID){
		confSpecROS.id = receivedConfSpec->id;
		confSpecROS.sensorSpec = receivedConfSpec->sensorSpec;
		confSpecROS.commSpec = receivedConfSpec->commSpec;
		confSpecROS.appSpec = receivedConfSpec->appSpec;
	}
}

void obuInterfaceConnection::selfMobCallback(const icps::VehicleInfo::ConstPtr& msg){
	INFMSG("selfMobCallback\n");
	TLVMsg_VEHICLE_INFO tlvObject;
	//FIXME: This is difficult case: ConstPtr is boost shareptr const, use get() to get raw pointer

	char gpsSample[26];
	ROSMsgHelper::gpsSampleTLVFromROSMsg(gpsSample, &((icps::VehicleInfo*) msg.get())->gpsSample);
	tlvObject.set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(gpsSample);
	char* tlv = tlvObject.toTLV();
	INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
	INFMSG("%s\n", tlvObject.toString().c_str());
	sendTLV(tlv);
}

void obuInterfaceConnection::onSocketError(){
	node.shutdown();
}
