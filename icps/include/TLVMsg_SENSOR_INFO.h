//description: SENSOR_INFO, (SMARTPHONE/OBU/ALIX -> LAPTOP)

#ifndef TLVMsg_SENSOR_INFO_H_
#define TLVMsg_SENSOR_INFO_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_DEVICE_ID.h"
#include "TLVClass_SENSOR_VALUE_LIST.h"

class TLVMsg_SENSOR_INFO{
public:
	static const TLVType type = TLVType_SENSOR_INFO;
	int length;

	TLVClass_DEVICE_ID m_DEVICE_ID;
	TLVClass_SENSOR_VALUE_LIST m_SENSOR_VALUE_LIST;

public:
	TLVMsg_SENSOR_INFO(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_DEVICE_ID::type){
			if(m_DEVICE_ID.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_SENSOR_VALUE_LIST::type){
			if(m_SENSOR_VALUE_LIST.parseTLV(curTLV) == false){
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
		m_DEVICE_ID.appendTLV(curAddress);
		curAddress += m_DEVICE_ID.getTLVSize();
		m_SENSOR_VALUE_LIST.appendTLV(curAddress);
		curAddress += m_SENSOR_VALUE_LIST.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_DEVICE_ID.getTLVSize();
		retSize += m_SENSOR_VALUE_LIST.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (SENSOR_INFO, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_DEVICE_ID = m_DEVICE_ID.toString(tabIndex+1);
		ss << child_m_DEVICE_ID;
		string child_m_SENSOR_VALUE_LIST = m_SENSOR_VALUE_LIST.toString(tabIndex+1);
		ss << child_m_SENSOR_VALUE_LIST;
		return ss.str();
	}

	void addElement_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_Vec(){TLVClass_GPS_SAMPLE _element; m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.push_back(_element);}

	uint64_t get_m_DEVICE_ID_V(){return m_DEVICE_ID.value;}
	void set_m_DEVICE_ID_V(uint64_t _value){m_DEVICE_ID.value = _value;}
	char* get_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(int _GPS_SAMPLE_Index){return m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.at(_GPS_SAMPLE_Index).value;}
	void set_m_SENSOR_VALUE_LIST_m_GPS_SAMPLE_vec_V(int _GPS_SAMPLE_Index, const char* _value){memcpy(m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.at(_GPS_SAMPLE_Index).value, _value, m_SENSOR_VALUE_LIST.m_GPS_SAMPLE_vec.at(_GPS_SAMPLE_Index).length);}
};
#endif
