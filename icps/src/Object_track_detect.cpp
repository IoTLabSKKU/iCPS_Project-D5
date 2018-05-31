#include "Object_track_detect.h"
#include <math.h>
#include <tf/transform_datatypes.h> // for tf library


//Hung
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
#define SpeedScale	(50.0)


Lidar_Object::Lidar_Object(float Prev_x , float Prev_y , float Prev_theta , float Dx , float Dy , double Dtheta , double Roll , double Pitch , double Yaw , double Yaw_degree  )
: fLatlon(0.0,0.0), cLatlon(0.0,0.0)// default
{
	dx = Dx;
	dy = Dy;
	dtheta = Dtheta;
	prev_pos_x = Prev_x;
	prev_pos_y = Prev_y;
	prev_theta = Prev_theta;
	roll = Roll;
	pitch = Pitch;
	yaw = Yaw;
	yaw_degree = Yaw_degree;


	fHeading = 0;
	cHeading = 0;
	cSpeed = 0;
	isFirstGPSSample = true;
	cYaw = 0;
	cPosX = 0;
	cPosY = 0;
	cur_pos_x = 0;
	cur_pos_y = 0;
	cur_heading = 0;


	selfMobilitySub = node.subscribe<icps::VehicleInfo>("icps/MobilityInfo/SelfMobility", 10, &Lidar_Object::vehInfoCallback, this);
	objectListPub = node.advertise<icps::ObjectList>("icps/SensorInfo/ObjectList", 10);
}

Lidar_Object::~Lidar_Object(){}


void Lidar_Object::vehInfoCallback(const icps::VehicleInfo::ConstPtr& _msg){
	icps::GPSSample gpsSample = ((icps::VehicleInfo*) _msg.get())->gpsSample;
	cLatlon.setLatDouble(gpsSample.latitude/LatlonScale);
	cLatlon.setLonDouble(gpsSample.longitude/LatlonScale);
	cHeading = gpsSample.heading/HeadingScale;
	cHeading = fmod(cHeading, 360);
	cHeading = cHeading > 180 ? 360 - cHeading : cHeading;
	cSpeed = (gpsSample.speed & 0x1FFF) * SpeedScale;

	if(isFirstGPSSample){
		fLatlon.setLatDouble(cLatlon.getLatDouble());
		fLatlon.setLonDouble(cLatlon.getLonDouble());
		fHeading = cHeading;
		isFirstGPSSample = false;
	}

	point fPos(fLatlon.getLonDouble(), fLatlon.getLatDouble());
	point cPos(cLatlon.getLonDouble(), cLatlon.getLatDouble());
	double distance = boost::geometry::distance(fPos, cPos, vincenty_type());

	double bearing = fLatlon.computeBearingTo(&cLatlon);
	double angle = Latlon::computeBearingDifferent(fHeading, bearing);
	cPosX = -distance*sin(DEG2RAD(angle));
	cPosY = distance*cos(DEG2RAD(angle));
	cYaw = -cHeading;
	INFMSG("Distance %f, fHeading %f, cHeading %f, Bearing %f, Angle %f, (%f, %f, %f, %f)\n", distance, fHeading, cHeading, bearing, angle, cPosX, cPosY, cSpeed, cYaw);

	//Interact with Howon code
	Set_Lidar_cur_posx(cPosX);
	Set_Lidar_cur_posy(cPosY);
	Set_Lidar_cur_heading(cYaw);
	Set_Lidar_cur_speed(cSpeed);

	transform.setOrigin(tf::Vector3(cPosX, cPosY, 0.0) );
	transform.setRotation(tf::Quaternion(0, 0, cYaw, 1) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "selfVehicleRelative"));
}

icps::ObjectInfo Lidar_Object::createObjectInfo(double objectDistance, double objectAngle, double objectID, double objectWidth, double objectLength){
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

double Lidar_Object::Set_threshold(double cur_vehicle_speed)
{	//40km /h ==> 11.11m/s  , 20km/h ==> 5.555m/s
	//25hz ==> 0.04s , 11.11 * 0.04 ==> 44cm
	double threshold_value = 0.0;
	if(cur_vehicle_speed <= 5.555 && cur_vehicle_speed >= 0.0)
		threshold_value = 0.20;
	else if(cur_vehicle_speed > 11.11 && cur_vehicle_speed > 5.555)
		threshold_value = 0.40;
	else
		threshold_value = 0.55;
	return threshold_value;
}

void Lidar_Object::Vehicle_moving(sensor_msgs::LaserScan *scan_msg_new , ros::Publisher *first_pcl_pub , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data , ros::Publisher *half_pcl_pub , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_1 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_2 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_3 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_4   , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_moving ,  ros::Publisher *last_pcl_pub)
{
	float Angular_Res = 0.1667;
	float FirstDegree = -5.0;
	float Distance;
	float Angle_In_Radian;
	float x_pos = 0.0;
	float y_pos = 0.0;


	for(int i = 0; i < scan_msg_new->ranges.size(); i++) //  0302 let's apply parallel and rotational translations
	{
		Distance = scan_msg_new->ranges[i];
		//Angle_In_Radian	 = ( FirstDegree + i*Angular_Res - Get_Lidar_cur_heading()) * M_PI / 180.0 ; // for demo
		Angle_In_Radian	 = ( FirstDegree + i*Angular_Res + Get_Lidar_cur_heading()) * M_PI / 180.0 ;
		x_pos =  Distance * cos(Angle_In_Radian) + Get_Lidar_cur_posx();
		y_pos =  Distance * sin(Angle_In_Radian) + Get_Lidar_cur_posy();						// for map building
		PCL_data->points.push_back (pcl::PointXYZ(x_pos, y_pos , 1.0));
	}

	count = ++count % frame_freq;
	int  frame_half_freq =  frame_freq/2;
	for(int i = 0; i < PCL_data->size(); i++){

		if(this->count % frame_freq == 1){
			if(i == 0)
				ROS_INFO(" (PCL_data_1) COUNT VALUE: %d ", count);
			PCL_data_1->points.push_back (pcl::PointXYZ(PCL_data->points[i].x , PCL_data->points[i].y , 1.0));
		}

		else if(this->count % frame_freq == frame_half_freq){
			if(i == 0)
				ROS_INFO(" (PCL_data_2) COUNT VALUE: %d", count);
			PCL_data_2->points.push_back (pcl::PointXYZ(PCL_data->points[i].x , PCL_data->points[i].y , 1.0));
		}


//		else if(this->count % frame_freq == frame_freq - 1){
//			if(i == 0)
//				ROS_INFO(" (PCL_data_3) COUNT VALUE: %d", count);
//			PCL_data_3->points.push_back (pcl::PointXYZ(PCL_data->points[i].x , PCL_data->points[i].y , 1.0));
//		}
	}

	last_pcl_pub->publish(*PCL_data);
	PCL_data->clear();
	ROS_INFO("count value : %d", count);
	if(count == frame_freq - 1){
		ROS_INFO("PUBLISHING PART VALUE %d , , DATA SIZE : %d " , count , PCL_data_1->size());
		first_pcl_pub->publish(*PCL_data_1);
		half_pcl_pub->publish(*PCL_data_2);
		//last_pcl_pub->publish(*PCL_data_3);
		PCL_data_1->clear();
		PCL_data_2->clear();
		//PCL_data_3->clear();
	}
}


void Lidar_Object::Set_prev_roll(double Roll){ this->roll = Roll; }
void Lidar_Object::Set_prev_pitch(double Pitch){	this->pitch = Pitch; }
void Lidar_Object::Set_prev_yaw(double Yaw){	this->yaw = Yaw; }
void Lidar_Object::Set_prev_yaw_degree(double Yaw_degree){	this->yaw_degree = Yaw_degree; }

double Lidar_Object::Get_prev_roll(){ return roll; }
double Lidar_Object::Get_prev_pitch(){	return pitch; }
double Lidar_Object::Get_prev_yaw(){	return yaw; }
double Lidar_Object::Get_prev_yaw_degree(){	return yaw_degree; }


void Lidar_Object::Set_Lidar_cur_speed(double Cur_vehicle_speed){cSpeed = Cur_vehicle_speed;}
double Lidar_Object::Get_Lidar_cur_speed(){return cSpeed;}
float Lidar_Object::Get_Lidar_cur_posx(){return cur_pos_x;}
float Lidar_Object::Get_Lidar_cur_posy(){return cur_pos_y;}
float Lidar_Object::Get_Lidar_cur_heading(){return cur_heading;}
float Lidar_Object::Get_prev_posx(){ return prev_pos_x; }
float Lidar_Object::Get_prev_posy(){ return prev_pos_y; }
float Lidar_Object::Get_prev_theta(){return prev_theta; }


void Lidar_Object::Set_prev_theta(float theta){	prev_theta = theta;}
void Lidar_Object::Set_dtheta_value(double theta_diff){	dtheta = theta_diff;}
void Lidar_Object::Set_Lidar_prev_posx(float prev_x)	{ prev_pos_x = prev_x; }
void Lidar_Object::Set_Lidar_prev_posy(float prev_y)	{ prev_pos_y = prev_y; }
void Lidar_Object::Set_Lidar_cur_posx(float Cur_pos_x)	{cur_pos_x = Cur_pos_x;}
void Lidar_Object::Set_Lidar_cur_posy(float Cur_pos_y)	{cur_pos_y = Cur_pos_y;}
void Lidar_Object::Set_Lidar_cur_heading(float Cur_heading){cur_heading = Cur_heading;}
void Lidar_Object::Set_dx_value(float Dx){	dx = Dx;}
void Lidar_Object::Set_dy_value(float Dy){	dy = Dy;}

float Lidar_Object::Get_dx_value(){	return dx;}
float Lidar_Object::Get_dy_value(){	return dy;}
double Lidar_Object::Get_dtheta_value(){	return dtheta; }

