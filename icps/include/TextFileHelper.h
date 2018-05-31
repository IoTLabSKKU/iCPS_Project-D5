/*
 * TextFileHelper.h
 *
 *  Created on: Jun 2, 2016
 *      Author: retry
 */

#ifndef TEXTFILEHELPER_H_
#define TEXTFILEHELPER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
/*
 *
 */
class TextFileHelper {
public:
	TextFileHelper(){}
	virtual ~TextFileHelper(){}

	static void writeSetLongToFile(string _fileName, set<long> *_setLong){
		ofstream myfile;
		myfile.open (_fileName);

		for(set<long>::iterator it=_setLong->begin(); it!=_setLong->end(); it++){
			myfile << *it << "\n";
		}
		myfile.close();
	}
	static void writeHashmapLongIntToFile(string _fileName, unordered_map<long, int> *_hashmapLongInt){
		ofstream myfile;
		myfile.open (_fileName);

		for(unordered_map<long, int>::iterator it=_hashmapLongInt->begin(); it!=_hashmapLongInt->end(); it++){
			myfile << it->first << "," << it->second << "\n";
		}
		myfile.close();
	}

	static void readHashmapLongStringFromFile(unordered_map<long, string> *_hashmapLongString, string _fileName){
		ifstream myfile;
		myfile.open(_fileName);
		string curLine;
		long key;

		while(getline(myfile, curLine)){
			istringstream iss(curLine);
			iss >> key;
			_hashmapLongString->insert(pair<long, string>(key, curLine));
		}
	}

	static void readHashmapStringLongFromFile(unordered_map<string, long> *_hashmap, string _fileName){
		ifstream myfile;
		myfile.open(_fileName);
		string curLine;
		string key;
		long value;

		while(getline(myfile, curLine)){
			istringstream iss(curLine);
			iss >> key;
			iss >> value;
			_hashmap->insert(pair<string, long>(key, value));
		}
	}
};

#endif /* TEXTFILEHELPER_H_ */
