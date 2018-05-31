#include "ros/ros.h"
#include "TLVMsg.h"
#include "ROSHelper.h"
#include "SocketHelper.h"


void sendingThreadHandler(int clientSockFD){

}


int main(int argc, char **argv){

	int serverPort = 6278;
	string serverIp = "127.0.0.1";

	int TLVHexSize;
	string latlonString;

	INFMSG("serverIp %s, port: %d.", serverIp.c_str(), serverPort);

	int sockfd = SocketHelper::createClientSockAndConnect(serverIp, serverPort);

	TLVMsg_FUNCTIONAL_SPEC funcSpec;
	funcSpec.set_m_DEVICE_ID_V(1234567890);
	funcSpec.set_m_DEVICE_TYPE_V(3);
	funcSpec.addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec();
	funcSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x0100010000000AFF);
	char* funcSpecTLV = funcSpec.toTLV();
	INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(funcSpecTLV));
	INFMSG("%s\n", funcSpec.toString().c_str());

	SocketHelper::sendTLV(sockfd, funcSpecTLV);

	char* confSpecTLVByte = SocketHelper::receiveTLV(sockfd);
	if(TLVHelper::isTLVType(confSpecTLVByte, TLVMsg_CONFIGURATION_SPEC::type)){
		TLVMsg_CONFIGURATION_SPEC confSpec;
		confSpec.parseTLV(confSpecTLVByte);
		INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(confSpecTLVByte));
		INFMSG("%s\n", confSpec.toString().c_str());
	}

	TLVMsg_SENSOR_INFO sensorInfo;
	sensorInfo.set_m_DEVICE_ID_V(1234567890);
	sensorInfo.addElement_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_Vec();
	char a[26] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5};
	sensorInfo.set_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(0, a);
	char* sensorInfoTLV = sensorInfo.toTLV();
	INFMSG("\n%s\n", TLVHelper::convertTLVToHexString(sensorInfoTLV));
	INFMSG("%s\n", sensorInfo.toString().c_str());

	SocketHelper::sendTLV(sockfd, sensorInfoTLV);

	while(true){
		char* tlv = SocketHelper::receiveTLV(sockfd);
		if(TLVHelper::isTLVType(tlv, TLVMsg_SENSOR_INFO_ACK::type)){
			TLVMsg_SENSOR_INFO_ACK ack;
			ack.parseTLV(tlv);
			INFMSG("%s\n", ack.toString().c_str());
		}else if(TLVHelper::isTLVType(tlv, TLVMsg_MOBILITY_SUMMARY::type)){
			TLVMsg_MOBILITY_SUMMARY tlvObject;
			tlvObject.parseTLV(tlv);
			INFMSG("%s\n", tlvObject.toString().c_str());
		}
	}


	SocketHelper::closeSocketAndPrintStatus(sockfd);
	return 0;
}


