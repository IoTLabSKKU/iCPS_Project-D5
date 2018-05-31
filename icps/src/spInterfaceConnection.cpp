/*
 * spInterfaceConnection.cpp
 *
 *  Created on: Jul 1, 2016
 *      Author: retry
 */

#include "spInterfaceConnection.h"


void spInterfaceConnection::start(){
	INFMSG("Connection start\n");

	selfSensorInfoPub = node.advertise<icps::GPSSample>("icps/SensorInfo/SelfGPSSample", 10, true);
	funcSpecPub = node.advertise<icps::FuncSpec>("icps/ConfInfo/FuncSpec", 10, true);
	confSpecSub = node.subscribe<icps::ConfSpec>("icps/ConfInfo/ConfSpec", 10, &spInterfaceConnection::confSpecCallback, this);
	mobilitySumSub = node.subscribe<icps::MobilitySummary>("icps/MobilityInfo/MobilitySummary", 10, &spInterfaceConnection::mobSumCallback, this);

	mState = waitFuncSpec;

	receiveTLV();
}

void spInterfaceConnection::receiveTLVCallback(std::vector<char> _tlv){
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

			ros::Rate r1(10);
			while(ros::ok()){
				r1.sleep();
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
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_MOBILITY_SUMMARY_ACK::type)){		//MOBILITY_SUMARY_ACK
			INFMSG("receiveTLVCallback mobSumAck\n");

			TLVMsg_MOBILITY_SUMMARY_ACK mobSumAck;
			mobSumAck.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", mobSumAck.toString().c_str());
			receiveTLV();
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_FUNCTIONAL_SPEC::type)){
			INFMSG("receiveTLVCallback funcSpec\n");

			TLVMsg_FUNCTIONAL_SPEC funcSpec;
			funcSpec.parseTLV(tlv);
			INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
			INFMSG("%s\n", funcSpec.toString().c_str());

			mDeviceID = funcSpec.get_m_DEVICE_ID_V();

			icps::FuncSpec funcSpecROS;
			ROSMsgHelper::funcSpecTLVToROSMsg(&funcSpecROS, &funcSpec);

			funcSpecPub.publish(funcSpecROS);

			ros::Rate r1(10);
			while(ros::ok()){
				r1.sleep();
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
			receiveTLV();
		}
	}
}

void spInterfaceConnection::laptopSettingsCallback(const icps::LaptopSettings::ConstPtr& msg){
	icps::LaptopSettings* laptopSettings = (icps::LaptopSettings*) msg.get();
}

void spInterfaceConnection::confSpecCallback(const icps::ConfSpec::ConstPtr& msg){
	INFMSG("confSpecCallback\n");

	icps::ConfSpec* receivedConfSpec = (icps::ConfSpec*) msg.get();
	if(receivedConfSpec->id == mDeviceID){
		confSpecROS.id = receivedConfSpec->id;
		confSpecROS.sensorSpec = receivedConfSpec->sensorSpec;
		confSpecROS.commSpec = receivedConfSpec->commSpec;
		confSpecROS.appSpec = receivedConfSpec->appSpec;
	}
}

void spInterfaceConnection::mobSumCallback(const icps::MobilitySummary::ConstPtr& msg){
	INFMSG("mobSumCallback\n");
	TLVMsg_MOBILITY_SUMMARY mobSum;
	//FIXME: This is difficult case: ConstPtr is boost shareptr const, use get() to get raw pointer

	ROSMsgHelper::mobSumTLVFromROSMsg(&mobSum, (icps::MobilitySummary*) msg.get());
	char* tlv = mobSum.toTLV();
	INFMSG("%s\n", TLVHelper::convertTLVToHexString(tlv));
	INFMSG("%s\n", mobSum.toString().c_str());
	sendTLV(tlv);
}

void spInterfaceConnection::onSocketError(){
	node.shutdown();
}
