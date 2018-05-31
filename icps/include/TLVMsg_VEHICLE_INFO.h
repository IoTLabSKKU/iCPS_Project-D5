//description: VEHICLE_INFO, (LAPTOP -> OBU/ALIX)

#ifndef TLVMsg_VEHICLE_INFO_H_
#define TLVMsg_VEHICLE_INFO_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_SELF_MOBILITY.h"

class TLVMsg_VEHICLE_INFO{
public:
	static const TLVType type = TLVType_VEHICLE_INFO;
	int length;

	TLVClass_SELF_MOBILITY m_SELF_MOBILITY;

public:
	TLVMsg_VEHICLE_INFO(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_SELF_MOBILITY::type){
			if(m_SELF_MOBILITY.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		return true;
	}

	char* toTLV(){
		int tlvSize = getTLVSize();
		char* retTLV = (char*) malloc(tlvSize*sizeof(char));
		appendTLV(retTLV);
		return retTLV;
	}

	void appendTLV(char* _address){
		char* curAddress = _address;
		TLVHelper::appendTLToByteArray(curAddress, type, getTLVSize() - TL_HEADER_SIZE);
		curAddress += TL_HEADER_SIZE;
		m_SELF_MOBILITY.appendTLV(curAddress);
		curAddress += m_SELF_MOBILITY.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_SELF_MOBILITY.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (VEHICLE_INFO, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_SELF_MOBILITY = m_SELF_MOBILITY.toString(tabIndex+1);
		ss << child_m_SELF_MOBILITY;
		return ss.str();
	}


	char* get_m_SELF_MOBILITY_m_GPS_SAMPLE_V(){return m_SELF_MOBILITY.m_GPS_SAMPLE.value;}
	void set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(const char* _value){memcpy(m_SELF_MOBILITY.m_GPS_SAMPLE.value, _value, m_SELF_MOBILITY.m_GPS_SAMPLE.length);}
};
#endif
