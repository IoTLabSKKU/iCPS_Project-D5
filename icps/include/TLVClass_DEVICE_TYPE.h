//description: DEVICE_TYPE, 

#ifndef TLVClass_DEVICE_TYPE_H_
#define TLVClass_DEVICE_TYPE_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"


class TLVClass_DEVICE_TYPE{
public:
	static const TLVType type = TLVType_DEVICE_TYPE;
	static const int length = 1;

	uint8_t value;

public:
	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		value = TLVHelper::getIntFromByteArray(_tlv + TL_HEADER_SIZE, length, false);

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
		TLVHelper::appendIntToByteArray(curAddress, value, length);
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += length;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (DEVICE_TYPE, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		ss<<TAB_STRING[tabIndex+1];
		ss<<setfill('0')<<setw(2)<<hex<<uppercase<<+value<<" ("<<dec<<nouppercase<<+value<<")"<<endl;
		return ss.str();
	}


	uint8_t get_V(){return value;}
	void set_V(uint8_t _value){value = _value;}
};
#endif
