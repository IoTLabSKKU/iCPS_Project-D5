//description: MOBILITY_SUMMARY, (LAPTOP -> SMARTPHONE)

#ifndef TLVMsg_MOBILITY_SUMMARY_H_
#define TLVMsg_MOBILITY_SUMMARY_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_SELF_MOBILITY.h"
#include "TLVClass_RSU_LIST.h"
#include "TLVClass_NEIGHBOR_LIST.h"
#include "TLVClass_PEDESTRIAN_LIST.h"
#include "TLVClass_OBJECT_LIST.h"

class TLVMsg_MOBILITY_SUMMARY{
public:
	static const TLVType type = TLVType_MOBILITY_SUMMARY;
	int length;

	TLVClass_SELF_MOBILITY m_SELF_MOBILITY;
	TLVClass_RSU_LIST m_RSU_LIST;
	TLVClass_NEIGHBOR_LIST m_NEIGHBOR_LIST;
	TLVClass_PEDESTRIAN_LIST m_PEDESTRIAN_LIST;
	TLVClass_OBJECT_LIST m_OBJECT_LIST;

public:
	TLVMsg_MOBILITY_SUMMARY(){length = -1;}

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

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_RSU_LIST::type){
			if(m_RSU_LIST.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_NEIGHBOR_LIST::type){
			if(m_NEIGHBOR_LIST.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_PEDESTRIAN_LIST::type){
			if(m_PEDESTRIAN_LIST.parseTLV(curTLV) == false){
				return false;
			}
		}else{
			return false;
		}

		curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
		if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_OBJECT_LIST::type){
			if(m_OBJECT_LIST.parseTLV(curTLV) == false){
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
		m_RSU_LIST.appendTLV(curAddress);
		curAddress += m_RSU_LIST.getTLVSize();
		m_NEIGHBOR_LIST.appendTLV(curAddress);
		curAddress += m_NEIGHBOR_LIST.getTLVSize();
		m_PEDESTRIAN_LIST.appendTLV(curAddress);
		curAddress += m_PEDESTRIAN_LIST.getTLVSize();
		m_OBJECT_LIST.appendTLV(curAddress);
		curAddress += m_OBJECT_LIST.getTLVSize();
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		retSize += m_SELF_MOBILITY.getTLVSize();
		retSize += m_RSU_LIST.getTLVSize();
		retSize += m_NEIGHBOR_LIST.getTLVSize();
		retSize += m_PEDESTRIAN_LIST.getTLVSize();
		retSize += m_OBJECT_LIST.getTLVSize();
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (MOBILITY_SUMMARY, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		string child_m_SELF_MOBILITY = m_SELF_MOBILITY.toString(tabIndex+1);
		ss << child_m_SELF_MOBILITY;
		string child_m_RSU_LIST = m_RSU_LIST.toString(tabIndex+1);
		ss << child_m_RSU_LIST;
		string child_m_NEIGHBOR_LIST = m_NEIGHBOR_LIST.toString(tabIndex+1);
		ss << child_m_NEIGHBOR_LIST;
		string child_m_PEDESTRIAN_LIST = m_PEDESTRIAN_LIST.toString(tabIndex+1);
		ss << child_m_PEDESTRIAN_LIST;
		string child_m_OBJECT_LIST = m_OBJECT_LIST.toString(tabIndex+1);
		ss << child_m_OBJECT_LIST;
		return ss.str();
	}

	void addElement_m_RSU_LIST_m_RSU_INFO_vec_Vec(){TLVClass_RSU_INFO _element; m_RSU_LIST.m_RSU_INFO_vec.push_back(_element);}
	void addElement_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_Vec(){TLVClass_NEIGHBOR_INFO _element; m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.push_back(_element);}
	void addElement_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_Vec(){TLVClass_PEDESTRIAN_INFO _element; m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.push_back(_element);}
	void addElement_m_OBJECT_LIST_m_OBJECT_INFO_vec_Vec(){TLVClass_OBJECT_INFO _element; m_OBJECT_LIST.m_OBJECT_INFO_vec.push_back(_element);}

	char* get_m_SELF_MOBILITY_m_GPS_SAMPLE_V(){return m_SELF_MOBILITY.m_GPS_SAMPLE.value;}
	void set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(const char* _value){memcpy(m_SELF_MOBILITY.m_GPS_SAMPLE.value, _value, m_SELF_MOBILITY.m_GPS_SAMPLE.length);}
	char* get_m_RSU_LIST_m_RSU_INFO_vec_V(int _RSU_INFO_Index){return m_RSU_LIST.m_RSU_INFO_vec.at(_RSU_INFO_Index).value;}
	void set_m_RSU_LIST_m_RSU_INFO_vec_V(int _RSU_INFO_Index, const char* _value){memcpy(m_RSU_LIST.m_RSU_INFO_vec.at(_RSU_INFO_Index).value, _value, m_RSU_LIST.m_RSU_INFO_vec.at(_RSU_INFO_Index).length);}
	uint64_t get_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(int _NEIGHBOR_INFO_Index){return m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.at(_NEIGHBOR_INFO_Index).m_VEHICLE_ID.value;}
	void set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(int _NEIGHBOR_INFO_Index, uint64_t _value){m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.at(_NEIGHBOR_INFO_Index).m_VEHICLE_ID.value = _value;}
	char* get_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(int _NEIGHBOR_INFO_Index){return m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.at(_NEIGHBOR_INFO_Index).m_GPS_SAMPLE.value;}
	void set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(int _NEIGHBOR_INFO_Index, const char* _value){memcpy(m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.at(_NEIGHBOR_INFO_Index).m_GPS_SAMPLE.value, _value, m_NEIGHBOR_LIST.m_NEIGHBOR_INFO_vec.at(_NEIGHBOR_INFO_Index).m_GPS_SAMPLE.length);}
	uint64_t get_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(int _PEDESTRIAN_INFO_Index){return m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_PEDESTRIAN_ID.value;}
	void set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(int _PEDESTRIAN_INFO_Index, uint64_t _value){m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_PEDESTRIAN_ID.value = _value;}
	char* get_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(int _PEDESTRIAN_INFO_Index){return m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.value;}
	void set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(int _PEDESTRIAN_INFO_Index, const char* _value){memcpy(m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.value, _value, m_PEDESTRIAN_LIST.m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.length);}
	uint64_t get_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_OBJECT_ID_V(int _OBJECT_INFO_Index){return m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_OBJECT_ID.value;}
	void set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_OBJECT_ID_V(int _OBJECT_INFO_Index, uint64_t _value){m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_OBJECT_ID.value = _value;}
	char* get_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_GPS_SAMPLE_V(int _OBJECT_INFO_Index){return m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_GPS_SAMPLE.value;}
	void set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_GPS_SAMPLE_V(int _OBJECT_INFO_Index, const char* _value){memcpy(m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_GPS_SAMPLE.value, _value, m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_GPS_SAMPLE.length);}
	uint64_t get_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_DIMENSION_V(int _OBJECT_INFO_Index){return m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_DIMENSION.value;}
	void set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_DIMENSION_V(int _OBJECT_INFO_Index, uint64_t _value){m_OBJECT_LIST.m_OBJECT_INFO_vec.at(_OBJECT_INFO_Index).m_DIMENSION.value = _value;}
};
#endif
