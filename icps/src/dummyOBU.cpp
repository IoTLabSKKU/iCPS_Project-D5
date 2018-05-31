#include "ros/ros.h"
#include "TLVMsg.h"
#include "ROSHelper.h"
#include "SocketHelper.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "dummyOBU");
	ros::NodeHandle n;
	int serverPort = 5100;
	string serverIp = "127.0.0.1";

	int TLVHexSize;
	string latlonString;

	INFMSG("serverIp %s, port: %d.", serverIp.c_str(), serverPort);

	int sockfd = SocketHelper::createClientSockAndConnect(serverIp, serverPort);

	TLVMsg_FUNCTIONAL_SPEC funcSpec;
	funcSpec.set_m_DEVICE_ID_V(2);
	funcSpec.set_m_DEVICE_TYPE_V(2);
	funcSpec.addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec();
	funcSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x01000A00000002FF);
	char* funcSpecTLV = funcSpec.toTLV();
	INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(funcSpecTLV));
	INFMSG("%s\n", funcSpec.toString().c_str());

	SocketHelper::sendTLV(sockfd, funcSpecTLV);

	char* confSpecTLVByte = SocketHelper::receiveTLV(sockfd);
	if(TLVHelper::isTLVType(confSpecTLVByte, TLVMsg_CONFIGURATION_SPEC::type)){
		TLVMsg_CONFIGURATION_SPEC confSpec;
		confSpec.parseTLV(confSpecTLVByte);
		INFMSG("%s\n", confSpec.toString().c_str());
	}

	for(int i=0; i<100; i++){

		TLVMsg_SENSOR_INFO sensorInfo;
		sensorInfo.set_m_DEVICE_ID_V(2);
		sensorInfo.addElement_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_Vec();
		unsigned char a[26] = {0x00, 0x00, 0x01, 0x61, 0x8C, 0xC2, 0xEC, 0x1C,
				0x15, 0x00, 0x2D, 0x22, 0x4C, 0xF0, 0x84, 0x66,
				0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x04, 0xB0};
		a[10] += i;
		sensorInfo.set_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(0,(char*)a);
		char* sensorInfoTLV = sensorInfo.toTLV();
		INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(sensorInfoTLV));
		INFMSG("%s\n", sensorInfo.toString().c_str());

		SocketHelper::sendTLV(sockfd, sensorInfoTLV);


		TLVMsg_REMOTE_INFO remoteInfo;
		remoteInfo.addElement_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_Vec();
		remoteInfo.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(0, 4);
		unsigned char b[26] = {0x00, 0x00, 0x01, 0x61, 0x8C, 0xC2, 0xEC, 0x1C,
				0x15, 0x00, 0x29, 0x8A, 0x4C, 0xF0, 0x80, 0x6A,
				0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x04, 0xB0};
		b[10] += i;
		remoteInfo.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(0, (char*)b);
		char* remoteInfoTLV = remoteInfo.toTLV();
		INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(remoteInfoTLV));
		INFMSG("%s\n", remoteInfo.toString().c_str());

		SocketHelper::sendTLV(sockfd, remoteInfoTLV);


		TLVMsg_REMOTE_INFO remoteInfo2;
		remoteInfo2.addElement_m_RSU_LIST_m_RSU_INFO_vec_Vec();
		remoteInfo2.addElement_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_Vec();
		unsigned char c[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05,
					0x15, 0x00, 0x2C, 0x3C, 0x4C, 0xF0, 0x88, 0x4E};
		remoteInfo2.set_m_RSU_LIST_m_RSU_INFO_vec_V(0, (char*)c);
		remoteInfo2.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(0, 6);
		unsigned char d[26] = {0x00, 0x00, 0x01, 0x61, 0x8C, 0xC2, 0xEC, 0x1C,
				0x15, 0x00, 0x29, 0x80, 0x4C, 0xF0, 0x85, 0x1A,
				0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x04, 0xB0};
		d[10] += i;
		remoteInfo2.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(0, (char*)d);
		char* remoteInfo2TLV = remoteInfo2.toTLV();
		INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(remoteInfo2TLV));
		INFMSG("%s\n", remoteInfo2.toString().c_str());

		SocketHelper::sendTLV(sockfd, remoteInfo2TLV);

		ros::Rate(1).sleep();
	}


	while(true){
		char* tlv = SocketHelper::receiveTLV(sockfd);
		if(TLVHelper::isTLVType(tlv, TLVMsg_SENSOR_INFO_ACK::type)){
			TLVMsg_SENSOR_INFO_ACK ack;
			ack.parseTLV(tlv);
			INFMSG("%s\n", ack.toString().c_str());
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_REMOTE_INFO_ACK::type)){
			TLVMsg_REMOTE_INFO_ACK ack;
			ack.parseTLV(tlv);
			INFMSG("%s\n", ack.toString().c_str());
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_VEHICLE_INFO::type)){
			TLVMsg_VEHICLE_INFO tlvObject;
			tlvObject.parseTLV(tlv);
			INFMSG("%s\n", tlvObject.toString().c_str());
		}
	}


	SocketHelper::closeSocketAndPrintStatus(sockfd);
	return 0;
}


