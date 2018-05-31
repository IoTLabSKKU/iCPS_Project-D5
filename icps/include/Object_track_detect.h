/*
 * object_track_detect.h
 *
 *  Created on: Jan 23, 2018
 *      Author: nsslab
 */

#ifndef OBJECT_TRACK_DETECT_H_
#define OBJECT_TRACK_DETECT_H_

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <nav_msgs/Odometry.h> //for Subscriber of Odom Data !


#include <pcl_ros/point_cloud.h>   // adding for point cloud data structure
#include <pcl/point_types.h>       // adding for point cloud data structure


//Hung
#include "icps/VehicleInfo.h"
#include "icps/ObjectList.h"
#include "Latlon.h"
#include "ROSHelper.h"
#include <tf/transform_broadcaster.h>


using namespace std;
class Lidar_Object {
private:
	double  roll, pitch, yaw , yaw_degree;
	double  	dx ;
	double 	dy ;
	double 	prev_pos_x ;
	double   prev_pos_y ;
	double   cur_pos_x;
	double   cur_pos_y;
	double   cur_heading;
	double 	prev_theta ;
	double   dtheta ;
	pair<float,float> pair_degree_dist;
	std::vector< pair<float ,float> > vector_degree_dist;
	std::vector< pair<int ,int> > vector_segment_groups;

	std::vector<int> moving_Obj_index_container; // 0308


	int 	count = 0; // for plotting several set of pointcloud (testing) 0304
	int   frame_freq = 25 * 5; //every 5 secs


	//Hung
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
	double	cSpeed;

	tf::TransformBroadcaster br;
	tf::Transform transform;



public:


	Lidar_Object(float Prev_x , float Prev_y , float Prev_theta , float Dx , float Dy , double Dtheta , double Roll , double Pitch , double Yaw , double Yaw_degree  );// default
	~Lidar_Object();

	void Set_prev_roll(double Roll);
	void Set_prev_pitch(double Pitch);
	void Set_prev_yaw(double Yaw);
	void Set_prev_yaw_degree(double Yaw_degree);

	double Get_prev_roll();
	double Get_prev_pitch();
	double Get_prev_yaw();
	double Get_prev_yaw_degree();


	float  Get_angular_vel();	// just for test
	void   Set_angular_vel(float Angular_vel);	// just for test
	void   Set_dx_value(float Dx);				 // from kobuki or from OBU
	void   Set_dy_value(float Dy);


	double Get_dtheta_value();
	float Get_prev_theta();
	float Get_dx_value();
	float Get_dy_value();
	float Get_prev_posx();
	float Get_prev_posy();
	float Get_Lidar_cur_posx();
	float Get_Lidar_cur_posy();
	float Get_Lidar_cur_heading();

	void Set_Lidar_cur_heading(float Cur_heading);
	void Set_Lidar_cur_posx(float Cur_pos_x);
	void Set_Lidar_cur_posy(float Cur_pos_x);
	void Set_Lidar_prev_posx(float prev_x); // from kobuki or from OBU
	void Set_Lidar_prev_posy(float prev_y);
	void Set_prev_theta(float theta);
	void Set_dtheta_value(double theta_diff);
	void Vehicle_moving(sensor_msgs::LaserScan *scan_msg_new , ros::Publisher *first_pcl_pub , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data , ros::Publisher *half_pcl_pub , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_1 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_2 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_3 , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_4   , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_moving,  ros::Publisher *last_PCL_pub);
	void Vehicle_not_moving(sensor_msgs::LaserScan *scan_msg_new ,sensor_msgs::LaserScan *scan_msg_copied, ros::Publisher *Vehicle_not_moving_pub);
	//void Vehicle_moving(sensor_msgs::LaserScan *scan_msg_new ,sensor_msgs::LaserScan *scan_msg_copied, ros::Publisher *Vehicle_moving_pub , ros::Publisher *PCL_pub , pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);


	double Set_threshold(double cur_vehicle_speed);
	void Set_Lidar_cur_speed(double Cur_vehicle_speed);
	double Get_Lidar_cur_speed();
	//Hung
	void vehInfoCallback(const icps::VehicleInfo::ConstPtr& _msg);
	icps::ObjectInfo createObjectInfo(double objectDistance, double objectAngle, double objectID, double objectWidth, double objectLength);

};



#endif /* OBJECT_TRACK_DETECT_H_ */
