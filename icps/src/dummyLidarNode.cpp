#include "ros/ros.h"
#include "TLVMsg.h"
#include "SocketHelper.h"
#include "icps/VehicleInfo.h"
#include "icps/ObjectList.h"
#include "Latlon.h"
#include <math.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/strategies/geographic/distance_vincenty.hpp>

typedef  boost::geometry:: model::point <double, 2, boost::geometry::cs::geographic <boost::geometry::degree > > point;
typedef  boost::geometry::srs::spheroid <double> stype;
typedef  boost::geometry:: strategy :: distance ::vincenty <stype > vincenty_type;

#define LatlonScale	(10000000.0)
#define HeadingScale	(80.0)
#define ObjectDimensionScale	(100.0)

class LidarNode{
public:
	LidarNode(): fLatlon(0.0,0.0), cLatlon(0.0,0.0){
		fHeading = 0;
		cHeading = 0;
		isFirstGPSSample = true;

		selfMobilitySub = node.subscribe<icps::VehicleInfo>("icps/MobilityInfo/SelfMobility", 10, &LidarNode::vehInfoCallback, this);
		objectListPub = node.advertise<icps::ObjectList>("icps/SensorInfo/ObjectList", 10);
	}

private:
	ros::NodeHandle node;
	ros::Publisher objectListPub;
	ros::Subscriber selfMobilitySub;

	Latlon	fLatlon;
	Latlon	cLatlon;
	double	fHeading;
	bool	isFirstGPSSample;

	double 	cHeading;

	double	cPosX;
	double	cPosY;
	double	cYaw;

	void vehInfoCallback(const icps::VehicleInfo::ConstPtr& _msg){
		icps::GPSSample gpsSample = ((icps::VehicleInfo*) _msg.get())->gpsSample;
		cLatlon.setLatDouble(gpsSample.latitude/LatlonScale);
		cLatlon.setLonDouble(gpsSample.longitude/LatlonScale);
		cHeading = gpsSample.heading/HeadingScale;
		cHeading = fmod(cHeading, 360);
		cHeading = cHeading > 180 ? 360 - cHeading : cHeading;

		if(isFirstGPSSample){
			fLatlon.setLatDouble(cLatlon.getLatDouble());
			fLatlon.setLonDouble(cLatlon.getLonDouble());
			fHeading = cHeading;
			isFirstGPSSample = false;
		}

		point fPos(fLatlon.getLonDouble(), fLatlon.getLatDouble());
		point cPos(cLatlon.getLonDouble(), cLatlon.getLatDouble());
		double distance1 = boost::geometry::distance(fPos, cPos, vincenty_type());

		double distance = fLatlon.computeDistanceTo(&cLatlon);
		double bearing = fLatlon.computeBearingTo(&cLatlon);
		double angle = Latlon::computeBearingDifferent(fHeading, bearing);
		cPosX = -distance1*sin(DEG2RAD(angle));
		cPosY = distance1*cos(DEG2RAD(angle));
		cYaw = -cHeading;
		INFMSG("GPSSample(%d,%d), cLatlon(%f,%f)", gpsSample.latitude, gpsSample.longitude, cLatlon.getLatDouble(), cLatlon.getLonDouble());
		INFMSG("Distance %f, Distance1 %f, fHeading %f, cHeading %f, Bearing %f, Angle %f, (%f, %f, %f)\n", distance, distance1, fHeading, cHeading, bearing, angle, cPosX, cPosY, cYaw);




		double objectDistance, objectAngle, objectID, objectWidth, objectLength;



		icps::ObjectList objectList;
		icps::ObjectInfo objectInfo = createObjectInfo(10, 90, 1, 3, 4);

		objectList.objectList.push_back(objectInfo);

		objectListPub.publish(objectList);
	}

	icps::ObjectInfo createObjectInfo(double objectDistance, double objectAngle, double objectID, double objectWidth, double objectLength){
		icps::ObjectInfo objectInfo;

		objectInfo.objectID = objectID;
		ros::Time now = ros::Time::now();
		objectInfo.gpsSample.utcTime = now.sec*1000 + now.nsec / 1000000;

		objectAngle = objectAngle < 0 ? objectAngle + 360 : objectAngle;
		Latlon objectLatlon = Latlon::getPointAtDistanceAndAngle(fLatlon, objectDistance, objectAngle);
		objectInfo.gpsSample.latitude = objectLatlon.getLatDouble()*LatlonScale;
		objectInfo.gpsSample.longitude = objectLatlon.getLonDouble()*LatlonScale;
		objectInfo.gpsSample.elevation = 0;
		objectInfo.gpsSample.accuracy = 0;
		objectInfo.gpsSample.heading = 0;
		objectInfo.gpsSample.speed = 0;
		objectInfo.width = objectWidth * ObjectDimensionScale;
		objectInfo.length = objectLength * ObjectDimensionScale;

		return objectInfo;
	}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "dummyLidarNode");


	LidarNode lidarNode;

	ros::spin();

	return 0;
}


