#include "ros/ros.h"
#include "TLVMsg.h"
#include "SocketHelper.h"

int main(int argc, char **argv){
	char* tlv;

	TLVMsg_FUNCTIONAL_SPEC funcSpec;
	funcSpec.set_m_DEVICE_ID_V(1);
	funcSpec.set_m_DEVICE_TYPE_V(3);
	funcSpec.addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec();
	funcSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x0100010000000AFF);
	tlv = funcSpec.toTLV();

	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", funcSpec.toString().c_str());

	funcSpec.set_m_DEVICE_ID_V(2);
	funcSpec.set_m_DEVICE_TYPE_V(2);
	funcSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x01000A00000002FF);
	tlv = funcSpec.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", funcSpec.toString().c_str());



	TLVMsg_CONFIGURATION_SPEC confSpec;
	confSpec.addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec();
	confSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x0100010000000A01);
	tlv = confSpec.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", confSpec.toString().c_str());

	confSpec.set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(0, 0x01000A0000000201);
	tlv = confSpec.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", confSpec.toString().c_str());



	TLVMsg_SENSOR_INFO sensorInfo;
	sensorInfo.set_m_DEVICE_ID_V(1);
	sensorInfo.addElement_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_Vec();
	char selfGPS[26];
	TLVHelper::appendIntToByteArray(selfGPS, 1518485040156, 8); //Feb 13, 2018, 01:24 (10:26 KST)
	TLVHelper::appendIntToByteArray(selfGPS + 8, 352333090, 4);	//NSSLab location
	TLVHelper::appendIntToByteArray(selfGPS + 12, 1290830950, 4);
	TLVHelper::appendIntToByteArray(selfGPS + 16, 20, 2);
	TLVHelper::appendIntToByteArray(selfGPS + 18, 0, 4);
	TLVHelper::appendIntToByteArray(selfGPS + 22, 100, 2);
	TLVHelper::appendIntToByteArray(selfGPS + 24, 1200, 2);
	sensorInfo.set_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(0, selfGPS);
	tlv = sensorInfo.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", sensorInfo.toString().c_str());


	TLVMsg_SENSOR_INFO_ACK sensorInfoAck;
	sensorInfoAck.set_m_RESPONSE_CODE_V(1);
	tlv = sensorInfoAck.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", sensorInfoAck.toString().c_str());


	TLVMsg_VEHICLE_INFO vehInfo;
	vehInfo.set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(selfGPS);
	tlv = vehInfo.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", vehInfo.toString().c_str());


	TLVMsg_VEHICLE_INFO_ACK vehInfoAck;
	vehInfoAck.set_m_RESPONSE_CODE_V(1);
	tlv = vehInfoAck.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", vehInfoAck.toString().c_str());


	TLVMsg_REMOTE_INFO remoteInfoNeighbor;
	remoteInfoNeighbor.addElement_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_Vec();
	remoteInfoNeighbor.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(0, 1);
	char neighborGPS[26];
	TLVHelper::appendIntToByteArray(neighborGPS, 1518485040156, 8); //Feb 13, 2018, 01:24 (10:26 KST)
	TLVHelper::appendIntToByteArray(neighborGPS + 8, 352332170, 4);	//NSSLab location
	TLVHelper::appendIntToByteArray(neighborGPS + 12, 1290829930, 4);
	TLVHelper::appendIntToByteArray(neighborGPS + 16, 20, 2);
	TLVHelper::appendIntToByteArray(neighborGPS + 18, 0, 4);
	TLVHelper::appendIntToByteArray(neighborGPS + 22, 100, 2);
	TLVHelper::appendIntToByteArray(neighborGPS + 24, 1200, 2);
	remoteInfoNeighbor.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(0, neighborGPS);
	tlv = remoteInfoNeighbor.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", remoteInfoNeighbor.toString().c_str());


	TLVMsg_REMOTE_INFO remoteInfoRSU;
	char rsu[16];
	TLVHelper::appendIntToByteArray(rsu, 1, 8);
	TLVHelper::appendIntToByteArray(rsu + 8, 352332860, 4);	//NSSLab location
	TLVHelper::appendIntToByteArray(rsu + 12, 1290831950, 4);
	remoteInfoRSU.addElement_m_RSU_LIST_m_RSU_INFO_vec_Vec();
	remoteInfoRSU.set_m_RSU_LIST_m_RSU_INFO_vec_V(0, rsu);
	remoteInfoRSU.addElement_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_Vec();
	remoteInfoRSU.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(0, 1);
	char pedestrianGPS[26];
	TLVHelper::appendIntToByteArray(pedestrianGPS, 1518485040156, 8); //Feb 13, 2018, 01:24 (10:26 KST)
	TLVHelper::appendIntToByteArray(pedestrianGPS + 8, 352332160, 4);	//NSSLab location
	TLVHelper::appendIntToByteArray(pedestrianGPS + 12, 1290831130, 4);
	TLVHelper::appendIntToByteArray(pedestrianGPS + 16, 20, 2);
	TLVHelper::appendIntToByteArray(pedestrianGPS + 18, 0, 4);
	TLVHelper::appendIntToByteArray(pedestrianGPS + 22, 100, 2);
	TLVHelper::appendIntToByteArray(pedestrianGPS + 24, 1200, 2);
	remoteInfoRSU.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(0, pedestrianGPS);
	tlv = remoteInfoRSU.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", remoteInfoRSU.toString().c_str());


	TLVMsg_REMOTE_INFO_ACK remoteInfoAck;
	remoteInfoAck.set_m_RESPONSE_CODE_V(1);
	tlv = remoteInfoAck.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", remoteInfoAck.toString().c_str());


	TLVMsg_MOBILITY_SUMMARY mobSum;
	mobSum.set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(selfGPS);
	mobSum.addElement_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_Vec();
	mobSum.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(0, 1);
	mobSum.set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(0, neighborGPS);
	mobSum.addElement_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_Vec();
	mobSum.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(0, 1);
	mobSum.set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(0, pedestrianGPS);
	mobSum.addElement_m_RSU_LIST_m_RSU_INFO_vec_Vec();
	mobSum.set_m_RSU_LIST_m_RSU_INFO_vec_V(0, rsu);
	mobSum.addElement_m_OBJECT_LIST_m_OBJECT_INFO_vec_Vec();
	mobSum.set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_OBJECT_ID_V(0, 1);
	char objectPosition[26];
	TLVHelper::appendIntToByteArray(objectPosition, 1518485040156, 8); //Feb 13, 2018, 01:24 (10:26 KST)
	TLVHelper::appendIntToByteArray(objectPosition + 8, 352333310, 4);	//NSSLab location
	TLVHelper::appendIntToByteArray(objectPosition + 12, 1290830300, 4);
	TLVHelper::appendIntToByteArray(objectPosition + 16, 0, 2);
	TLVHelper::appendIntToByteArray(objectPosition + 18, 0, 4);
	TLVHelper::appendIntToByteArray(objectPosition + 22, 0, 2);
	TLVHelper::appendIntToByteArray(objectPosition + 24, 0, 2);
	mobSum.set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_GPS_SAMPLE_V(0, objectPosition);
	tlv = mobSum.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", mobSum.toString().c_str());


	TLVMsg_MOBILITY_SUMMARY_ACK mobSumAck;
	mobSumAck.set_m_RESPONSE_CODE_V(1);
	tlv = mobSumAck.toTLV();
	printf("\n%s\n", TLVHelper::convertTLVToHexString(tlv));
	printf("%s\n", mobSumAck.toString().c_str());


	long a = 0x0102030405060708;
	long a1 = TLVHelper::getIntFromLong(a, 1, 2, false);
	printf("%lx", a1);

	return 0;
}


