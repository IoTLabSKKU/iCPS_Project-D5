//description: GPS_SAMPLE, 

#ifndef TLVClass_GPS_SAMPLE_H_
#define TLVClass_GPS_SAMPLE_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"


class TLVClass_GPS_SAMPLE{
public:
	static const TLVType type = TLVType_GPS_SAMPLE;
	static const int length = 26;

	char value[26];

public:
	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		TLVHelper::copyByteArray(value, _tlv + TL_HEADER_SIZE, length);

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
		TLVHelper::copyByteArray(curAddress, value, length);
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
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (GPS_SAMPLE, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		ss<<TAB_STRING[tabIndex+1];
		char* value_hexarray = TLVHelper::convertByteArrayToHexString(value, length);
		string value_string(value_hexarray);
		ss<<value_string<<endl;
		free(value_hexarray);
		return ss.str();
	}


	char* get_V(){return value;}
	void set_V(const char* _value){memcpy(value, _value, length);}
};
#endif
