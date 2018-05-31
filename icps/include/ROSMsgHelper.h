/*
 * ROSMsgHelper.h
 *
 *  Created on: Jan 25, 2018
 *      Author: retry
 */

#ifndef SRC_ICPS_INCLUDE_ROSMSGHELPER_H_
#define SRC_ICPS_INCLUDE_ROSMSGHELPER_H_

#include "ROSHelper.h"
#include "TLVHelper.h"
#include "icps/GPSSample.h"
#include "icps/MobilitySummary.h"
#include "icps/FuncSpec.h"
#include "icps/ConfSpec.h"

class ROSMsgHelper {
public:
	static void gpsSampleTLVToROSMsg(icps::GPSSample* _dest, const char* _source){
		_dest->utcTime = TLVHelper::getIntFromByteArray(_source, 8, false);
		_dest->latitude = TLVHelper::getIntFromByteArray(_source+8, 4, true);
		_dest->longitude = TLVHelper::getIntFromByteArray(_source+12, 4, true);
		_dest->elevation = TLVHelper::getIntFromByteArray(_source+16, 2, true);
		_dest->accuracy = TLVHelper::getIntFromByteArray(_source+18, 4, true);
		_dest->speed = TLVHelper::getIntFromByteArray(_source+22, 2, true);
		_dest->heading = TLVHelper::getIntFromByteArray(_source+24, 2, true);
	}

	static void gpsSampleTLVFromROSMsg(char* _dest, icps::GPSSample* _source){
		TLVHelper::appendIntToByteArray(_dest, _source->utcTime, 8);
		TLVHelper::appendIntToByteArray(_dest+8, _source->latitude, 4);
		TLVHelper::appendIntToByteArray(_dest+12, _source->longitude, 4);
		TLVHelper::appendIntToByteArray(_dest+16, _source->elevation, 2);
		TLVHelper::appendIntToByteArray(_dest+18, _source->accuracy, 4);
		TLVHelper::appendIntToByteArray(_dest+22, _source->speed, 2);
		TLVHelper::appendIntToByteArray(_dest+24, _source->heading, 2);
	}

	static void rsuInfoTLVToROSMsg(icps::RSUInfo* _dest, const char* _source){
		_dest->RSUID = TLVHelper::getIntFromByteArray(_source, 8, false);
		_dest->latitude = TLVHelper::getIntFromByteArray(_source+8, 4, true);
		_dest->longitude = TLVHelper::getIntFromByteArray(_source+12, 4, true);
	}

	static void rsuInfoTLVFromROSMsg(char* _dest, icps::RSUInfo* _source){
		TLVHelper::appendIntToByteArray(_dest, _source->RSUID, 8);
		TLVHelper::appendIntToByteArray(_dest+8, _source->latitude, 4);
		TLVHelper::appendIntToByteArray(_dest+12, _source->longitude, 4);
	}

	static void mobSumTLVFromROSMsg(TLVMsg_MOBILITY_SUMMARY* _dest, icps::MobilitySummary* _source){
		char selfGPSSample[26];
		gpsSampleTLVFromROSMsg(selfGPSSample, &_source->selfMobility);
		_dest->set_m_SELF_MOBILITY_m_GPS_SAMPLE_V(selfGPSSample);
		for(int i=0; i<_source->neighborList.size(); i++){
			_dest->addElement_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_Vec();
			_dest->set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_VEHICLE_ID_V(i, _source->neighborList.at(i).vehicleID);
			char gpsSample[26];
			gpsSampleTLVFromROSMsg(gpsSample, &_source->neighborList.at(i).gpsSample);
			_dest->set_m_NEIGHBOR_LIST_m_NEIGHBOR_INFO_vec_m_GPS_SAMPLE_V(i, gpsSample);
		}

		for(int i=0; i<_source->pedestrianList.size(); i++){
			_dest->addElement_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_Vec();
			_dest->set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_PEDESTRIAN_ID_V(i, _source->pedestrianList.at(i).pedestrianID);
			char gpsSample[26];
			gpsSampleTLVFromROSMsg(gpsSample, &_source->pedestrianList.at(i).gpsSample);
			_dest->set_m_PEDESTRIAN_LIST_m_PEDESTRIAN_INFO_vec_m_GPS_SAMPLE_V(i, gpsSample);
		}

		for(int i=0; i<_source->rsuList.size(); i++){
			_dest->addElement_m_RSU_LIST_m_RSU_INFO_vec_Vec();
			char rsuInfo[16];
			rsuInfoTLVFromROSMsg(rsuInfo, &_source->rsuList.at(i));
			_dest->set_m_RSU_LIST_m_RSU_INFO_vec_V(i, rsuInfo);
		}

		for(int i=0; i<_source->objectList.size(); i++){
			_dest->addElement_m_OBJECT_LIST_m_OBJECT_INFO_vec_Vec();
			_dest->set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_OBJECT_ID_V(i, _source->objectList.at(i).objectID);
			char gpsSample[26];
			gpsSampleTLVFromROSMsg(gpsSample, &_source->objectList.at(i).gpsSample);
			_dest->set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_GPS_SAMPLE_V(i, gpsSample);
			long dimension = 0;
			TLVHelper::appendIntToLong(&dimension, _source->objectList.at(i).width, 4, 4);
			TLVHelper::appendIntToLong(&dimension, _source->objectList.at(i).length, 0, 4);
			_dest->set_m_OBJECT_LIST_m_OBJECT_INFO_vec_m_DIMENSION_V(i, dimension);
		}
	}

	static void funcSpecTLVToROSMsg(icps::FuncSpec* _dest, TLVMsg_FUNCTIONAL_SPEC* _source){
		_dest->id = _source->get_m_DEVICE_ID_V();
		_dest->type = _source->get_m_DEVICE_TYPE_V();
		for(int i=0; i<_source->m_SENSOR_SPEC_LIST.m_SENSOR_SPEC_vec.size(); i++){
			long specLong = _source->get_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(i);
			icps::SensorSpec specROS;
			specROS.type = TLVHelper::getIntFromLong(specLong, 7, 1, false);
			specROS.rate = TLVHelper::getIntFromLong(specLong, 5, 2, false);
			specROS.accuracy = TLVHelper::getIntFromLong(specLong, 1, 4, false);
			specROS.inUse = TLVHelper::getIntFromLong(specLong, 0, 1, false);
			_dest->sensorSpec.push_back(specROS);
		}

		for(int i=0; i<_source->m_COMM_SPEC_LIST.m_COMM_SPEC_vec.size(); i++){
			long specLong = _source->get_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_V(i);
			icps::CommSpec specROS;
			specROS.type = TLVHelper::getIntFromLong(specLong, 3, 1, false);
			specROS.rate = TLVHelper::getIntFromLong(specLong, 1, 2, false);
			specROS.inUse = TLVHelper::getIntFromLong(specLong, 0, 1, false);
			_dest->commSpec.push_back(specROS);
		}

		for(int i=0; i<_source->m_APP_SPEC_LIST.m_APP_SPEC_vec.size(); i++){
			long specLong = _source->get_m_APP_SPEC_LIST_m_APP_SPEC_vec_V(i);
			icps::AppSpec specROS;
			specROS.type = TLVHelper::getIntFromLong(specLong, 7, 1, false);
			specROS.spec = TLVHelper::getIntFromLong(specLong, 1, 6, false);
			specROS.inUse = TLVHelper::getIntFromLong(specLong, 0, 1, false);
			_dest->appSpec.push_back(specROS);
		}
	}

	static void confSpecFromMsg(TLVMsg_CONFIGURATION_SPEC* _dest, icps::ConfSpec* _source){
		for(int i=0; i<_source->sensorSpec.size(); i++){
			icps::SensorSpec specROS = _source->sensorSpec.at(i);
			long specLong = 0;
			TLVHelper::appendIntToLong(&specLong, specROS.type, 7, 1);
			TLVHelper::appendIntToLong(&specLong, specROS.rate, 5, 2);
			TLVHelper::appendIntToLong(&specLong, specROS.accuracy, 1, 4);
			TLVHelper::appendIntToLong(&specLong, specROS.inUse, 0, 1);
			_dest->addElement_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_Vec();
			_dest->set_m_SENSOR_SPEC_LIST_m_SENSOR_SPEC_vec_V(i, specLong);
		}

		for(int i=0; i<_source->commSpec.size(); i++){
			icps::CommSpec specROS = _source->commSpec.at(i);
			long specLong = 0;
			TLVHelper::appendIntToLong(&specLong, specROS.type, 3, 1);
			TLVHelper::appendIntToLong(&specLong, specROS.rate, 1, 2);
			TLVHelper::appendIntToLong(&specLong, specROS.inUse, 0, 1);
			_dest->addElement_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_Vec();
			_dest->set_m_COMM_SPEC_LIST_m_COMM_SPEC_vec_V(i, specLong);
		}

		for(int i=0; i<_source->appSpec.size(); i++){
			icps::AppSpec specROS = _source->appSpec.at(i);
			long specLong = 0;
			TLVHelper::appendIntToLong(&specLong, specROS.type, 7, 1);
			TLVHelper::appendIntToLong(&specLong, specROS.spec, 1, 6);
			TLVHelper::appendIntToLong(&specLong, specROS.inUse, 0, 1);
			_dest->addElement_m_APP_SPEC_LIST_m_APP_SPEC_vec_Vec();
			_dest->set_m_APP_SPEC_LIST_m_APP_SPEC_vec_V(i, specLong);
		}
	}
};


#endif /* SRC_ICPS_INCLUDE_ROSMSGHELPER_H_ */
