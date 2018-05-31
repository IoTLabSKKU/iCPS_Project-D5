//description: NEIGHBOR_INFO, 

#ifndef TLVClass_NEIGHBOR_INFO_H_
#define TLVClass_NEIGHBOR_INFO_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_VEHICLE_ID.h"
#include "TLVClass_GPS_SAMPLE.h"

class TLVClass_NEIGHBOR_INFO{
public:
	static const TLVType type = TLVType_NEIGHBOR_INFO;
	int length;

	TLVClass_VEHICLE_ID m_VEHICLE_ID;
	TLVClass_GPS_SAMPLE m_GPS_SAMPLE;

public:
	TLVClass_NEIGHBOR_INFO(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_VEHICLE_ID::type){
			if(m_VEHICLE_ID.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_GPS_SAMPLE::type){
			if(m_GPS_SAMPLE.parseTLV(curTLV) == false){
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
		m_VEHICLE_ID.appendTLV(curAddress);
		curAddress += m_VEHICLE_ID.getTLVSize();
		m_GPS_SAMPLE.appendTLV(curAddress);
		curAddress += m_GPS_SAMPLE.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_VEHICLE_ID.getTLVSize();
		retSize += m_GPS_SAMPLE.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (NEIGHBOR_INFO, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_VEHICLE_ID = m_VEHICLE_ID.toString(tabIndex+1);
		ss << child_m_VEHICLE_ID;
		string child_m_GPS_SAMPLE = m_GPS_SAMPLE.toString(tabIndex+1);
		ss << child_m_GPS_SAMPLE;
		return ss.str();
	}


	uint64_t get_m_VEHICLE_ID_V(){return m_VEHICLE_ID.value;}
	void set_m_VEHICLE_ID_V(uint64_t _value){m_VEHICLE_ID.value = _value;}
	char* get_m_GPS_SAMPLE_V(){return m_GPS_SAMPLE.value;}
	void set_m_GPS_SAMPLE_V(const char* _value){memcpy(m_GPS_SAMPLE.value, _value, m_GPS_SAMPLE.length);}
};
#endif
