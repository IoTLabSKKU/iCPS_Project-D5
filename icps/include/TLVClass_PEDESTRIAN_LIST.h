//description: PEDESTRIAN_LIST, 

#ifndef TLVClass_PEDESTRIAN_LIST_H_
#define TLVClass_PEDESTRIAN_LIST_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_PEDESTRIAN_INFO.h"

class TLVClass_PEDESTRIAN_LIST{
public:
	static const TLVType type = TLVType_PEDESTRIAN_LIST;
	int length;

	vector<TLVClass_PEDESTRIAN_INFO> m_PEDESTRIAN_INFO_vec;

public:
	TLVClass_PEDESTRIAN_LIST(){length = -1;}

	bool parseTLV(char* _tlv){
		assert(TLVHelper::getTLVTypeField(_tlv) == type);
		int curLength = TLVHelper::getTLVLengthField(_tlv);
		assert(length == -1 ? true : (length == curLength));
		length = curLength;
		char* curTLV = _tlv + TL_HEADER_SIZE;
		char* curAddress = _tlv + TL_HEADER_SIZE;

		while(true){
			char* lastAddress = curAddress;
			curTLV = TLVHelper::getSubTLVOnebyone(_tlv, &curAddress, false);
			if(curTLV != NULL){
				if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_PEDESTRIAN_INFO::type){
					TLVClass_PEDESTRIAN_INFO curTLVObject;
					if(curTLVObject.parseTLV(curTLV) == false){
						return false;
					}
					m_PEDESTRIAN_INFO_vec.push_back(curTLVObject);
				}else{
					curAddress = lastAddress;
					break;
				}
			}else{
				curAddress = lastAddress;
				break;
			}
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
		for(int i=0; i<m_PEDESTRIAN_INFO_vec.size(); i++){
			m_PEDESTRIAN_INFO_vec.at(i).appendTLV(curAddress);
			curAddress += m_PEDESTRIAN_INFO_vec.at(i).getTLVSize();
		}
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		for(int i=0; i<m_PEDESTRIAN_INFO_vec.size(); i++){
			retSize += m_PEDESTRIAN_INFO_vec.at(i).getTLVSize();
		}
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (PEDESTRIAN_LIST, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		for(int i=0; i<m_PEDESTRIAN_INFO_vec.size(); i++){
			string child_m_PEDESTRIAN_INFO_vec = m_PEDESTRIAN_INFO_vec.at(i).toString(tabIndex+1);
			ss << child_m_PEDESTRIAN_INFO_vec;
		}
		return ss.str();
	}

	void addElement_m_PEDESTRIAN_INFO_vec_Vec(){TLVClass_PEDESTRIAN_INFO _element; m_PEDESTRIAN_INFO_vec.push_back(_element);}

	uint64_t get_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(int _PEDESTRIAN_INFO_Index){return m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_PEDESTRIAN_ID.value;}
	void set_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(int _PEDESTRIAN_INFO_Index, uint64_t _value){m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_PEDESTRIAN_ID.value = _value;}
	char* get_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(int _PEDESTRIAN_INFO_Index){return m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.value;}
	void set_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(int _PEDESTRIAN_INFO_Index, const char* _value){memcpy(m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.value, _value, m_PEDESTRIAN_INFO_vec.at(_PEDESTRIAN_INFO_Index).m_GPS_SAMPLE.length);}
};
#endif
