//description: VEHICLE_INFO_ACK, (OBU/ALIX -> LAPTOP)

#ifndef TLVMsg_VEHICLE_INFO_ACK_H_
#define TLVMsg_VEHICLE_INFO_ACK_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_RESPONSE_CODE.h"

class TLVMsg_VEHICLE_INFO_ACK{
public:
	static const TLVType type = TLVType_VEHICLE_INFO_ACK;
	int length;

	TLVClass_RESPONSE_CODE m_RESPONSE_CODE;

public:
	TLVMsg_VEHICLE_INFO_ACK(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_RESPONSE_CODE::type){
			if(m_RESPONSE_CODE.parseTLV(curTLV) == false){
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
		m_RESPONSE_CODE.appendTLV(curAddress);
		curAddress += m_RESPONSE_CODE.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_RESPONSE_CODE.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (VEHICLE_INFO_ACK, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_RESPONSE_CODE = m_RESPONSE_CODE.toString(tabIndex+1);
		ss << child_m_RESPONSE_CODE;
		return ss.str();
	}


	uint8_t get_m_RESPONSE_CODE_V(){return m_RESPONSE_CODE.value;}
	void set_m_RESPONSE_CODE_V(uint8_t _value){m_RESPONSE_CODE.value = _value;}
};
#endif
