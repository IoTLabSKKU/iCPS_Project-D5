/*
 * Latlon.h
 *
 *  Created on: Jun 2, 2016
 *      Author: retry
 */

#ifndef LATLON_H_
#define LATLON_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCALE_FACTOR_LATLON	(10000000.0)
#define DOUBLE2LONG(x)	((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define DEG2RAD(a)	(a*M_PI/180)
#define	RAD2DEG(a)	(a*180/M_PI)
#define EARTH_RADIUS_IN_METERS	(6372797.560856)

struct StructLatlon{
	long lat;
	long lon;
};
typedef struct StructLatlon LatlonStruct;

/*
 *
 */
class Latlon {
private:
	long mLatInt;
	long mLonInt;
	double mLatDouble;
	double mLonDouble;
public:
	Latlon(long _latInt, long _lonInt){
		mLatInt = _latInt;
		mLonInt = _lonInt;
		mLatDouble = mLatInt / SCALE_FACTOR_LATLON;
		mLonDouble = mLonInt / SCALE_FACTOR_LATLON;
	}
	Latlon(double _latDouble, double _lonDouble){
		mLatDouble = _latDouble;
		mLonDouble = _lonDouble;
		mLatInt = DOUBLE2LONG(mLatDouble*SCALE_FACTOR_LATLON);
		mLonInt = DOUBLE2LONG(mLonDouble*SCALE_FACTOR_LATLON);
	}
	Latlon(Latlon* _originalPoint){
		mLatInt = _originalPoint->mLatInt;
		mLonInt = _originalPoint->mLonInt;
		mLatDouble = _originalPoint->mLatDouble;
		mLonDouble = _originalPoint->mLonDouble;
	}
	virtual ~Latlon(){}

	Latlon & operator=(const Latlon &_inputLatlon){
		mLatInt = _inputLatlon.mLatInt;
		mLonInt = _inputLatlon.mLonInt;
		mLatDouble = _inputLatlon.mLatDouble;
		mLonDouble = _inputLatlon.mLonDouble;

		return *this;
	}

	long getLatInt(){return mLatInt;}
	long getLonInt(){return mLonInt;}
	double getLatDouble(){return mLatDouble;}
	double getLonDouble(){return mLonDouble;}

	void setLonDouble(double _lon){mLonDouble = _lon; mLonInt = DOUBLE2LONG(mLonDouble*SCALE_FACTOR_LATLON);}
	void setLatDouble(double _lat){mLatDouble = _lat; mLatInt = DOUBLE2LONG(mLatDouble*SCALE_FACTOR_LATLON);}

	char* toString(){
		char* retString = (char*)malloc(24*sizeof(char));
		snprintf(retString, 23, "%.7f,%.7f", mLatDouble, mLonDouble);
		return retString;
	}

	LatlonStruct* getLatlonStruct(){
		LatlonStruct *retStruct = new LatlonStruct();
		retStruct->lat = mLatInt;
		retStruct->lon = mLonInt;
		return retStruct;
	}

	double computeDistanceTo(Latlon *_targetPoint){
		double lat1 = DEG2RAD(mLatDouble);
		double lon1 = DEG2RAD(mLonDouble);
		double lat2 = DEG2RAD(_targetPoint->mLatDouble);
		double lon2 = DEG2RAD(_targetPoint->mLonDouble);

		double latAngle = (lat1-lat2)/2;
		double lngAngle = (lon1-lon2)/2;

		// Calculation by applying Haversine formula
		double dist = 2*EARTH_RADIUS_IN_METERS*asin( sqrt( sin(latAngle)*sin(latAngle) + cos(lat1)*cos(lat2)*sin(lngAngle)*sin(lngAngle)));

		return dist;
	}
	double computeBearingTo(Latlon *_targetPoint){
		double lat1 = DEG2RAD(mLatDouble);
		double lon1 = DEG2RAD(mLonDouble);
		double lat2 = DEG2RAD(_targetPoint->mLatDouble);
		double lon2 = DEG2RAD(_targetPoint->mLonDouble);

		double bearingInRad = atan2(sin(lon2 - lon1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1));
		double retBearing = RAD2DEG(bearingInRad);
		if (retBearing < 0)
			retBearing = 360 + retBearing;

		return retBearing;
	}

	static double computeBearingDifferent(double _b1, double _b2){
		return fmod((fmod((_b1 - _b2), 360) + 540), 360) - 180;
	}

	static Latlon getPointAtDistanceAndAngle(Latlon _source, double _distance, double _angle){
		double latRad = DEG2RAD(_source.mLatDouble);
		double lonRad = DEG2RAD(_source.mLonDouble);

		double lat2Rad = asin(sin(latRad)*cos(_distance/EARTH_RADIUS_IN_METERS) +
				cos(latRad)*sin(_distance/EARTH_RADIUS_IN_METERS)*cos(DEG2RAD(_angle)));
		double lon2Rad = lonRad + atan2(sin(DEG2RAD(_angle))*sin(_distance/EARTH_RADIUS_IN_METERS)*cos(latRad),
				cos(_distance/EARTH_RADIUS_IN_METERS) - sin(latRad)*sin(lat2Rad));
		Latlon ret(RAD2DEG(lat2Rad), RAD2DEG(lon2Rad));

		return ret;
	}
};

#endif /* LATLON_H_ */
