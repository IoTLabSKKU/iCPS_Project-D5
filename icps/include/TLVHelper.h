/*
 * TLVHelper.h
 *
 *  Created on: Jan 24, 2018
 *      Author: retry
 */

#ifndef TLVHELPER_H_
#define TLVHELPER_H_

#include "ROSHelper.h"

#define TLV_TYPE_SIZE	3
#define TLV_LENGTH_SIZE	3
#define TL_HEADER_SIZE	6

#define TLV_MAX_TYPE_VALUE	(0xFFFFFF)
#define TLV_MAX_LENGTH_VALUE	(0xFFFFFF)

#define LatlonScale	(10000000.0)
#define HeadingScale	(80.0)
#define ObjectDimensionScale	(100.0)
#define SpeedScale	(50.0)

using namespace std;

static char HexNumberToCharMap[16] = {
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};
static char HexCharToNumberMap[103] = {
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -1, -1, -1, -1, -1, -1,
		-1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, 10, 11, 12, 13, 14, 15
};
static const string TAB_STRING[10] = {"", "  ", "    ", "      ", "        ", "          ", "            ", "              ", "                ", "                  "};

class TLVHelper {
public:


	/********** Basic TLV Function **********/
	static int getTLVTypeField(const char* _TLV){
		if(_TLV == NULL){
			ERRMSG("Null input");
			return 0;
		}

		return getIntFromByteArray(_TLV, TLV_TYPE_SIZE, false);
	}

	static int getTLVLengthField(const char* _TLV){
		if(_TLV == NULL){
			ERRMSG("Null input");
			return 0;
		}

		return getIntFromByteArray(_TLV + TLV_TYPE_SIZE, TLV_LENGTH_SIZE, false);
	}

	static void appendTLVTypeField(char* _byteArray, int _length){
		if(_byteArray == NULL){
			ERRMSG("Null input");
			return;
		}
		if(_length > TLV_MAX_TYPE_VALUE){
			ERRMSG("Length exceeds max length value");
			return;
		}

		appendIntToByteArray(_byteArray, _length, TLV_TYPE_SIZE);
	}

	static void appendTLVLengthField(char* _byteArray, int _length){
		if(_byteArray == NULL){
			ERRMSG("Null input");
			return;
		}
		if(_length > TLV_MAX_LENGTH_VALUE){
			ERRMSG("Length exceeds max length value");
			return;
		}

		appendIntToByteArray(_byteArray, _length, TLV_LENGTH_SIZE);
	}

	static void appendTLToByteArray(char* _byteArray, int _type, int _length){
		// This is bridge function so don't need to check the input condition
		appendTLVTypeField(_byteArray, _type);
		appendTLVLengthField(_byteArray + TLV_TYPE_SIZE, _length);
	}

	static int getTLVSize(const char* _TLV) {
		if(_TLV == NULL){
			ERRMSG("Null input");
			return 0;
		}

		return TL_HEADER_SIZE + getTLVLengthField(_TLV);
	}

	static char* convertTLVToHexString(const char* _tlv){
		if(_tlv == NULL){
			ERRMSG("Null input");
			return 0;
		}

		int size = getTLVSize(_tlv);
		char* hexString = convertByteArrayToHexString(_tlv, size);
		return hexString;
	}

	/********** Complex TLV Function **********/
	static bool isTLVType(const char* _TLV, int _type){
		int type = getTLVTypeField(_TLV);
		if(type == _type){
			return true;
		}

		return false;
	}

	static char* getSubTLVOnebyone(const char* _TLV, char** _curSubTLVAddress, bool _isCopy){
		if(_TLV == NULL){
			ERRMSG("Null input");
			return NULL;
		}

		int curTLVIndex = 0;
		int outerTLVLength = getTLVLengthField(_TLV) + TL_HEADER_SIZE;
		int curTLVSize;
		char* ret;

		if(*_curSubTLVAddress < _TLV + outerTLVLength){
			curTLVSize = getTLVSize(*_curSubTLVAddress);
			if(_isCopy == true){
				ret = (char*) malloc(curTLVSize*sizeof(char));
				memcpy(ret, *_curSubTLVAddress, curTLVSize);
			}else{
				ret = *_curSubTLVAddress;
			}

			*_curSubTLVAddress += curTLVSize;
			return ret;
		}

		return NULL;
	}


	/********** Byte Array Function **********/
	static long getIntFromByteArray(const char* _byteArray, short _size, bool _sign) {
		if(_byteArray == NULL){
			ERRMSG("Null input");
			return -1;
		}

		long retSign;
		long retUnsign;
		switch(_size){
		case 1:
			retUnsign = (long)_byteArray[0] & 0xFF;
			retSign = retUnsign | (retUnsign>0x7F?0xFFFFFFFFFFFFFF00:0);
			break;
		case 2:
			retUnsign = (((long)_byteArray[0] << 8) & 0xFF00) | ((long)_byteArray[1] & 0xFF);
			retSign = retUnsign | (retUnsign>0x7FFF?0xFFFFFFFFFFFF0000:0);
			break;
		case 3:
			retUnsign = (((long)_byteArray[0] << 16) & 0xFF0000) | (((long)_byteArray[1] << 8) & 0xFF00) | ((long)_byteArray[2] & 0xFF);
			retSign = retUnsign | (retUnsign>0x7FFFFF?0xFFFFFFFFFF000000:0);
			break;
		case 4:
			retUnsign = (((long)_byteArray[0] << 24) & 0xFF000000) | (((long)_byteArray[1] << 16) & 0xFF0000) |
					(((long)_byteArray[2] << 8) & 0xFF00) | ((long)_byteArray[3] & 0xFF);
			retSign = retUnsign | (retUnsign>0x7FFFFFFF?0xFFFFFFFF00000000:0);
			break;
		case 6:
			retUnsign = (((long)_byteArray[0] << 40) & 0xFF0000000000) | (((long)_byteArray[1] << 32) & 0xFF00000000) |
					(((long)_byteArray[2] << 24) & 0xFF000000) | (((long)_byteArray[3] << 16) & 0xFF0000) |
					(((long)_byteArray[4] << 8) & 0xFF00) | ((long)_byteArray[5] & 0xFF);
			retSign = retUnsign | (retUnsign>0x7FFFFFFFFFFF?0xFFFF000000000000:0);
			break;
		case 8:
			retUnsign = (((long)_byteArray[0] << 56) & 0xFF00000000000000) | (((long)_byteArray[1] << 48) & 0xFF000000000000) |
					(((long)_byteArray[2] << 40) & 0xFF0000000000) | (((long)_byteArray[3] << 32) & 0xFF00000000) |
					(((long)_byteArray[4] << 24) & 0xFF000000) | (((long)_byteArray[5] << 16) & 0xFF0000) |
					(((long)_byteArray[6] << 8) & 0xFF00) | ((long)_byteArray[7] & 0xFF);
			retSign = retUnsign;	//Not support unsigned long
			break;
		default:
			ERRMSG("Unsupported size");
			retUnsign = 0;
			retSign = 0;
			break;
		}

		if(_sign == true){
			return retSign;
		}else{
			return retUnsign;
		}
	}

	static void appendIntToByteArray(char* _byteArray, long _int, short _size){
		if(_byteArray == NULL){
			ERRMSG("Null input");
			return;
		}

		switch(_size){
		case 1:
			_byteArray[0] = _int & 0xFF;
			break;
		case 2:
			_byteArray[0] = (_int>>8) & 0xFF;
			_byteArray[1] = _int & 0xFF;
			break;
		case 3:
			_byteArray[0] = (_int>>16) & 0xFF;
			_byteArray[1] = (_int>>8) & 0xFF;
			_byteArray[2] = _int & 0xFF;
			break;
		case 4:
			_byteArray[0] = (_int>>24) & 0xFF;
			_byteArray[1] = (_int>>16) & 0xFF;
			_byteArray[2] = (_int>>8) & 0xFF;
			_byteArray[3] = _int & 0xFF;
			break;
		case 6:
			_byteArray[0] = (_int>>40) & 0xFF;
			_byteArray[1] = (_int>>32) & 0xFF;
			_byteArray[2] = (_int>>24) & 0xFF;
			_byteArray[3] = (_int>>16) & 0xFF;
			_byteArray[4] = (_int>>8) & 0xFF;
			_byteArray[5] = _int & 0xFF;
			break;
		case 8:
			_byteArray[0] = (_int>>56) & 0xFF;
			_byteArray[1] = (_int>>48) & 0xFF;
			_byteArray[2] = (_int>>40) & 0xFF;
			_byteArray[3] = (_int>>32) & 0xFF;
			_byteArray[4] = (_int>>24) & 0xFF;
			_byteArray[5] = (_int>>16) & 0xFF;
			_byteArray[6] = (_int>>8) & 0xFF;
			_byteArray[7] = _int & 0xFF;
			break;
		default:
			break;
		}
	}

	static long getIntFromLong(long _long, short _pos, short _size, bool _sign) {
		short bitPos = _pos*8;
		long retSign;
		long retUnsign;
		switch(_size){
		case 1:
			retUnsign = (_long >> bitPos) & 0xFF;
			retSign = retUnsign | (retUnsign>0x7F?0xFFFFFFFFFFFFFF00:0);
			break;
		case 2:
			retUnsign = (_long >> bitPos) & 0xFFFF;
			retSign = retUnsign | (retUnsign>0x7FFF?0xFFFFFFFFFFFF0000:0);
			break;
		case 3:
			retUnsign = (_long >> bitPos) & 0xFFFFFF;
			retSign = retUnsign | (retUnsign>0x7FFFFF?0xFFFFFFFFFF000000:0);
			break;
		case 4:
			retUnsign = (_long >> bitPos) & 0xFFFFFFFF;
			retSign = retUnsign | (retUnsign>0x7FFFFFFF?0xFFFFFFFF00000000:0);
			break;
		case 6:
			retUnsign = (_long >> bitPos) & 0xFFFFFFFFFFFF;
			retSign = retUnsign | (retUnsign>0x7FFFFFFFFFFF?0xFFFF000000000000:0);
			break;
		default:
			ERRMSG("Unsupported size");
			retUnsign = 0;
			retSign = 0;
			break;
		}

		if(_sign == true){
			return retSign;
		}else{
			return retUnsign;
		}
	}

	static void appendIntToLong(long* _long, long _int, short _pos, short _size){
		short bitPos = _pos*8;

		switch(_size){
		case 1:
			*_long |= (_int & 0xFF) << bitPos;
			break;
		case 2:
			*_long |= (_int & 0xFFFF) << bitPos;
			break;
		case 3:
			*_long |= (_int & 0xFFFFFF) << bitPos;
			break;
		case 4:
			*_long |= (_int & 0xFFFFFFFF) << bitPos;
			break;
		case 6:
			*_long |= (_int & 0xFFFFFFFFFFFF) << bitPos;
			break;
		default:
			break;
		}
	}

	static void copyByteArray(char* _dest, const char* _source, short _size){
		if(_dest == NULL || _source == NULL){
			ERRMSG("Null input");
			return;
		}

		for(int i=0; i<_size; i++){
			_dest[i] = _source[i];
		}
	}

	static bool compareByteArray(const char* _byteArray1, const char* _byteArray2, int _size) {
		if(_byteArray1 == NULL || _byteArray2 == NULL){
			ERRMSG("Null input");
			return false;
		}

		for(int i=0; i<_size; i++){
			if(_byteArray1[i] != _byteArray2[i]){
				return false;
			}
		}

		return true;
	}


	static char* convertByteArrayToHexString(const char* _byteArray, int _size) {
		if(_byteArray == NULL){
			ERRMSG("Null input");
			return NULL;
		}

		int hexStringSize = _size*2+1;
		char* ret = (char*) malloc(hexStringSize*sizeof(char));
		for(int i=0; i<_size; i++){
			ret[i*2] = convertHexNumberToChar((_byteArray[i] >> 4) & 0xf);
			ret[i*2+1] = convertHexNumberToChar(_byteArray[i] & 0xf);
		}

		ret[_size*2] = 0;	//put end string character in hex string

		return ret;
	}

	static char* convertHexStringToByteArray(const char* _hexString, int _size) {
		if(_hexString == NULL){
			ERRMSG("Null input");
			return NULL;
		}

		int byteArraySize = _size/2;
		char* ret = (char*) malloc(byteArraySize*sizeof(char));
		for(int i=0; i<byteArraySize; i++){
			ret[i] = ((convertHexCharToNumber(_hexString[i*2])<<4)&0xf0) | (convertHexCharToNumber(_hexString[i*2+1])&0x0f);
		}
		return ret;
	}

	/********** Hex and Char Function **********/
	static char convertHexNumberToChar(char _number){
		if(_number > 15 || _number < 0){
			ERRMSG( "Invalid hex digit number, value: %d", _number);
			return -1;
		}

		char ret = HexNumberToCharMap[(unsigned char)_number];
		return ret;
	}
	static char convertHexCharToNumber(char _char){
		if(_char < 48 || (_char > 57 && _char < 65) || (_char > 70 && _char < 97) || _char > 102){
			ERRMSG("Invalid hex digit char, ascii code: %d", _char);
			return -1;
		}

		char ret = HexCharToNumberMap[(unsigned char)_char];
		return ret;
	}
};

#endif /* TLVHELPER_H_ */
