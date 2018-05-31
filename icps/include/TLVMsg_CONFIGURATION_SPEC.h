//description: CONFIGURATION_SPEC, (LAPTOP -> SMARTPHONE/OBU/ALIX)

#ifndef TLVMsg_CONFIGURATION_SPEC_H_
#define TLVMsg_CONFIGURATION_SPEC_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_SENSOR_SPEC_LIST.h"
#include "TLVClass_COMM_SPEC_LIST.h"
#include "TLVClass_APP_SPEC_LIST.h"

class TLVMsg_CONFIGURATION_SPEC{
public:
	static const TLVType type = TLVType_CONFIGURATION_SPEC;
	int length;

	TLVClass_SENSOR_SPEC_LIST m_SENSOR_SPEC_LIST;
	TLVClass_COMM_SPEC_LIST m_COMM_SPEC_LIST;
	TLVClass_APP_SPEC_LIST m_APP_SPEC_LIST;

public:
	TLVMsg_CONFIGURATION_SPEC(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_SENSOR_SPEC_LIST::type){
			if(m_SENSOR_SPEC_LIST.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_COMM_SPEC_LIST::type){
			if(m_COMM_SPEC_LIST.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_APP_SPEC_LIST::type){
			if(m_APP_SPEC_LIST.parseTLV(curTLV) == false){
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
		m_SENSOR_SPEC_LIST.appendTLV(curAddress);
		curAddress += m_SENSOR_SPEC_LIST.getTLVSize();
		m_COMM_SPEC_LIST.appendTLV(curAddress);
		curAddress += m_COMM_SPEC_LIST.getTLVSize();
		m_APP_SPEC_LIST.appendTLV(curAddress);
		curAddress += m_APP_SPEC_LIST.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_SENSOR_SPEC_LIST.getTLVSize();
		retSize += m_COMM_SPEC_LIST.getTLVSize();
		retSize += m_APP_SPEC_LIST.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (CONFIGURATION_SPEC, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_SENSOR_SPEC_LIST = m_SENSOR_SPEC_LIST.toString(tabIndex+1);
		ss << child_m_SENSOR_SPEC_LIST;
		string child_m_COMM_SPEC_LIST = m_COMM_SPEC_LIST.toString(tabIndex+1);
		ss << child_m_COMM_SPEC_LIST;
		string child_m_APP_SPEC_LIST = m_APP_SPEC_LIST.toString(tabIndex+1);
		ss << child_m_APP_SPEC_LIST;
		return ss.str();
	}

	void addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec(){TLVClass_SENSOR_SPEC _element; m_SENSOR_SPEC_LIST.m_SENSOR_SPEC_vec.push_back(_element);}
	void addElement_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_Vec(){TLVClass_COMM_SPEC _element; m_COMM_SPEC_LIST.m_COMM_SPEC_vec.push_back(_element);}
	void addElement_m_APP_SPEC_LIST_m_APP_SPEC_vec_Vec(){TLVClass_APP_SPEC _element; m_APP_SPEC_LIST.m_APP_SPEC_vec.push_back(_element);}

	uint64_t get_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(int _SENSOR_SPEC_Index){return m_SENSOR_SPEC_LIST.m_SENSOR_SPEC_vec.at(_SENSOR_SPEC_Index).value;}
	void set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(int _SENSOR_SPEC_Index, uint64_t _value){m_SENSOR_SPEC_LIST.m_SENSOR_SPEC_vec.at(_SENSOR_SPEC_Index).value = _value;}
	uint32_t get_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_V(int _COMM_SPEC_Index){return m_COMM_SPEC_LIST.m_COMM_SPEC_vec.at(_COMM_SPEC_Index).value;}
	void set_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_V(int _COMM_SPEC_Index, uint32_t _value){m_COMM_SPEC_LIST.m_COMM_SPEC_vec.at(_COMM_SPEC_Index).value = _value;}
	uint64_t get_m_APP_SPEC_LIST_m_APP_SPEC_vec_V(int _APP_SPEC_Index){return m_APP_SPEC_LIST.m_APP_SPEC_vec.at(_APP_SPEC_Index).value;}
	void set_m_APP_SPEC_LIST_m_APP_SPEC_vec_V(int _APP_SPEC_Index, uint64_t _value){m_APP_SPEC_LIST.m_APP_SPEC_vec.at(_APP_SPEC_Index).value = _value;}
};
#endif
