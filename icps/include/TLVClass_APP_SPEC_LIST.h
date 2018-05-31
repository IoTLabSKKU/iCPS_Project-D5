//description: APP_SPEC_LIST, 

#ifndef TLVClass_APP_SPEC_LIST_H_
#define TLVClass_APP_SPEC_LIST_H_
#include <vector>
#include <stdint.h>
#include <sstream>
#include "TLVHelper.h"
#include "TLVTypeDefinition.h"

#include "TLVClass_APP_SPEC.h"

class TLVClass_APP_SPEC_LIST{
public:
	static const TLVType type = TLVType_APP_SPEC_LIST;
	int length;

	vector<TLVClass_APP_SPEC> m_APP_SPEC_vec;

public:
	TLVClass_APP_SPEC_LIST(){length = -1;}

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
				if(TLVHelper::getTLVTypeField(curTLV) == TLVClass_APP_SPEC::type){
					TLVClass_APP_SPEC curTLVObject;
					if(curTLVObject.parseTLV(curTLV) == false){
						return false;
					}
					m_APP_SPEC_vec.push_back(curTLVObject);
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
		for(int i=0; i<m_APP_SPEC_vec.size(); i++){
			m_APP_SPEC_vec.at(i).appendTLV(curAddress);
			curAddress += m_APP_SPEC_vec.at(i).getTLVSize();
		}
	}

	int getTLVSize(){
		int retSize = TL_HEADER_SIZE;
		for(int i=0; i<m_APP_SPEC_vec.size(); i++){
			retSize += m_APP_SPEC_vec.at(i).getTLVSize();
		}
		length = retSize - TL_HEADER_SIZE;
		return retSize;
	}

	string toString(int tabIndex=0){
		stringstream ss;
		if(tabIndex==0){ss<<endl;}
		string header;
		ss<<TAB_STRING[tabIndex];
		ss<<setfill('0')<<setw(6)<<hex<<uppercase<<type<<" "<<setfill('0')<<setw(6)<<hex<<uppercase<<length<<" (APP_SPEC_LIST, "<<dec<<nouppercase<<length<<")"<<endl;
		ss << header;
		for(int i=0; i<m_APP_SPEC_vec.size(); i++){
			string child_m_APP_SPEC_vec = m_APP_SPEC_vec.at(i).toString(tabIndex+1);
			ss << child_m_APP_SPEC_vec;
		}
		return ss.str();
	}

	void addElement_m_APP_SPEC_vec_Vec(){TLVClass_APP_SPEC _element; m_APP_SPEC_vec.push_back(_element);}

	uint64_t get_m_APP_SPEC_vec_V(int _APP_SPEC_Index){return m_APP_SPEC_vec.at(_APP_SPEC_Index).value;}
	void set_m_APP_SPEC_vec_V(int _APP_SPEC_Index, uint64_t _value){m_APP_SPEC_vec.at(_APP_SPEC_Index).value = _value;}
};
#endif
