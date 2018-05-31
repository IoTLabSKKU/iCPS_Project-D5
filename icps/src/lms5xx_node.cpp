///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2008, Morgan Quigley
// Copyright (C) 2011, Robert Huitl
// Copyright (C) 2015, Natan Biesmans
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   * Neither the name of KU Leuven University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
#include <cstdio>
#include <math.h>
#include <SickLMS5xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h> // for subscriber

//#include "sensor_msgs/PointCloud.h"

#include "Object_track_detect.h"
#include "geometry_msgs/Point32.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"

#include <algorithm>
#include <vector> // ==1==
#include <iostream>
#include <time.h>
#include <ctime>
#include <boost/thread/thread.hpp>
//#include <thread>
using namespace message_filters;
using namespace SickToolbox;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan , nav_msgs::Odometry> NoCloudSyncPolicy;

void publish_scan( 	sensor_msgs::LaserScan *scan_msg
		,ros::Publisher *pub, uint32_t *range_values,
		uint32_t n_range_values, uint32_t *intensity_values,
		uint32_t n_intensity_values, double scale, ros::Time start,
		double scan_time, bool inverted, float angle_min,
		float angle_max, std::string frame_id)
{

	float 		 FirstDegree = -5.0;
	size_t		 s = 0;
	size_t		 temp_s = 0;
	size_t  	 temp_dist;
	size_t  	 temp_short_dist;// nearer point
	size_t  	 temp_long_dist; // further point
	bool		 Dist_Inc_Check = false;
	double		 temp_range = 0;
	double		 Angular_Res;
	int			 count = 1;

	float 		 Intensity_Val =(float)0.0;

	pair<unsigned int,unsigned int> pair_temp;
	std::vector< pair<unsigned int,unsigned int> > vec;
	scan_time = 8.0 / 75;// * (0.1667/0.25); // 53.33 ms

	if(scan_time == (8.0 / 75))
		Angular_Res = 0.1667;

	else if(scan_time == (4.0 / 75))
		Angular_Res = 0.25;

	else if(scan_time == (2.0 / 75))
		Angular_Res = 0.5;

	if(scan_time == (1.0 / 75))
		Angular_Res = 1.0;
	// due to 50hz ==2==
	// The scan time is always 1/75 because that's how long it takes
	// for the mirror to rotate. If we have a higher resolution, the
	// SICKs interleave the readings, so the net result is we just
	// shift the measurements.
	// 0.5 degrees
	//scan_time = 2.0 / 75; // 26.66 ms

	static int scan_count = 0;
	//sensor_msgs::LaserScan scan_msg;
	//sensor_msgs::LaserScan scan_msg;// ANOTHER LASERSCAN OBJECT FOR COMPARISION WITH PRIVIOUS SCAN DATA ARRAY
	scan_msg->header.frame_id = frame_id;
	scan_count++;

	if(inverted) {
		scan_msg->angle_min = angle_max;
		scan_msg->angle_max = angle_min;
	} else {
		scan_msg->angle_min = angle_min;
		scan_msg->angle_max = angle_max;
	}

	scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min)/ (double)(n_range_values - 1);
	scan_msg->scan_time = scan_time;
	scan_msg->time_increment = scan_time / (2 * M_PI) * scan_msg->angle_increment;
	scan_msg->range_min = 0;
	scan_msg->range_max = 81.;
	scan_msg->ranges.resize(n_range_values);
	scan_msg->header.stamp = start;


	for(size_t i = 0; i < n_range_values ; i++) {
		scan_msg->ranges[i] = (float)range_values[i] * (float)scale;
	}

	float Min_val = scan_msg->ranges[0];
	float Min_val_Angle;

	for(size_t i = 0; i < n_range_values ; i++) {
		if(scan_msg->ranges[i] <= Min_val){
			Min_val = scan_msg->ranges[i];
			if(Min_val <= 0.1)
			{
				//	scan_msg->ranges[i] = (scan_msg->ranges[i+1] + scan_msg->ranges[i-1]) / 2.0;
				scan_msg->ranges[i] = scan_msg->ranges[i-1];
			}
			Min_val_Angle = FirstDegree + (i)*Angular_Res;
		}

	}

	scan_msg->intensities.resize(n_range_values);

	const clock_t begin_time = clock();

	while(s < n_range_values - 1)
	{
		temp_s = s;

		for(; s < n_range_values; s++){

			if(std::abs(scan_msg->ranges[s] - scan_msg->ranges[s+1]) >  0.5)
			{
				temp_dist = temp_short_dist;

				if(scan_msg->ranges[s] > scan_msg->ranges[s+1]){
					temp_short_dist = s + 1;
					temp_long_dist = s;
					Dist_Inc_Check  = false;
				}
				else{
					temp_short_dist = s;
					temp_long_dist = s + 1;
					Dist_Inc_Check  = true;
				}

				double Between_DIST = sqrt( pow(scan_msg->ranges[temp_dist],2) + pow(scan_msg->ranges[temp_short_dist],2) - 2*scan_msg->ranges[temp_dist]*scan_msg->ranges[temp_short_dist]*cos(Angular_Res*(std::abs(temp_short_dist-temp_dist)) * M_PI / 180.0) );
				double Between_DIST_1 = sqrt( pow(scan_msg->ranges[temp_long_dist],2) + pow(scan_msg->ranges[temp_short_dist],2) - 2*scan_msg->ranges[temp_long_dist]*scan_msg->ranges[temp_short_dist]*cos(Angular_Res*(std::abs(temp_short_dist-temp_long_dist)) * M_PI / 180.0) );

				if(Between_DIST_1 <= 0.2 && std::abs(scan_msg->ranges[temp_long_dist] - scan_msg->ranges[temp_short_dist]) <  0.5 )
				{
					if(Dist_Inc_Check == true)
					{
						scan_msg->intensities[temp_long_dist] = scan_msg->intensities[temp_short_dist];
						Intensity_Val = scan_msg->intensities[temp_long_dist];
					}

					else
					{
						scan_msg->intensities[temp_short_dist] = scan_msg->intensities[temp_long_dist];
						Intensity_Val = scan_msg->intensities[temp_short_dist];
					}

					s = s + 1;
					break;
				}

				else
				{
					Intensity_Val += 50.0;
					//if(Intensity_Val == 300) // for color clearness.
					//	Intensity_Val = 0.0;
					if(Dist_Inc_Check == true)
					{
						s = s + 1;
						break;
					}

					else // false 일 경우만 한번 더 체크 , 거리가 조금 떨어져있더라도 한물체인지 아닌지 확인
					{
						if(Between_DIST <= 0.2 && std::abs(scan_msg->ranges[temp_dist] - scan_msg->ranges[temp_short_dist]) <  0.5 )
						{	// 배경을 완벽한 방법으로 추출하지는 않음. 중간에 물체가 있으면 색깔이 바뀜.
							//if(temp_dist < temp_short_dist){ // temp 값이 항상 temp_short_dist 값보다는 작음
							scan_msg->intensities[temp_short_dist] = scan_msg->intensities[temp_dist];
							Intensity_Val = scan_msg->intensities[temp_short_dist];
							s = s+1;
							break;
						}
					}
				}

				if(s + 1 == n_range_values - 1)
				{
					s = s+1;
					break;
				}

			}

			else
			{ // color doesn't change
				scan_msg->intensities[s] = Intensity_Val;// First_Intensity_Val;
				scan_msg->intensities[s+1] = Intensity_Val;

				if(s + 1 == n_range_values - 1)
				{
					scan_msg->intensities[s + 1] = scan_msg->intensities[s];
					s = s+1;
					break;
				}
			}

		}
	}
	pub->publish(*scan_msg);
}

void Init_for_secondArr(sensor_msgs::LaserScan *scan_msg_copied  , uint32_t *range_values, // ==3==
		uint32_t n_range_values, ros::Time start,
		double scan_time, bool inverted, float angle_min,
		float angle_max, std::string frame_id )
{
	static int scan_count = 0;
	scan_time = 8.0 / 75;// * (0.1667/0.25); //

	scan_msg_copied->header.frame_id = frame_id;
	scan_count++;

	if(inverted) {
		scan_msg_copied->angle_min = angle_max;
		scan_msg_copied->angle_max = angle_min;
	} else {
		scan_msg_copied->angle_min = angle_min;
		scan_msg_copied->angle_max = angle_max;
	}

	scan_msg_copied->angle_increment = (scan_msg_copied->angle_max - scan_msg_copied->angle_min)/ (double)(n_range_values - 1);
	scan_msg_copied->scan_time = scan_time;
	scan_msg_copied->time_increment = scan_time / (2 * M_PI) * (scan_msg_copied->angle_increment);
	scan_msg_copied->range_min = 0;
	scan_msg_copied->range_max = 81.;
	scan_msg_copied->header.stamp = start;

	scan_msg_copied->ranges.resize(n_range_values);
	scan_msg_copied->intensities.resize(n_range_values);

	//pub_1->publish(*scan_msg_copied);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "sicklms5xx");
	string port;
	int baud;
	bool inverted = false;
	int angle;
	double resolution;
	std::string frame_id;
	std::string ip_add;
	double scan_time = 0;
	double angle_increment = 0;
	float angle_min = 0.0;
	float angle_max = 0.0;

	sensor_msgs::LaserScan Scan_msg;  // Scan_msg 를 메인에다가 추가 // 원래는 publish_scan 함수내에서만 정의되어있던 것!
	sensor_msgs::LaserScan Scan_msg_Copied;

	//	sensor_msgs::LaserScan* Scan_msg_Pointer = &Scan_msg;
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan> ("icps/scan", 1);


	//	ros::Publisher Polygon_pub = nh.advertise<geometry_msgs::PolygonStamped> ("polygon", 1); // for bounding box !!! 0205
	ros::Publisher Vehicle_moving_pub = nh.advertise<sensor_msgs::LaserScan> ("icps/Vehicle_moving_pub", 1);

//	ros::Publisher Moving_Obj_average_pub = nh.advertise<sensor_msgs::LaserScan> ("Moving_Obj_average", 1);
//	ros::Publisher Moving_Obj_radius_pub = nh.advertise<sensor_msgs::LaserScan> ("Moving_Obj_radius", 1);

	ros::Publisher first_pcl_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("icps/first_pcl_pub", 1);       //  for plotting of PointCloud data 0302 , declaration!
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data (new pcl::PointCloud<pcl::PointXYZ>);
	pcl_data->header.frame_id = "map";
	pcl_data->height = pcl_data->width = 1;


	ros::Publisher half_pcl_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("icps/half_pcl_pub", 1);       //  for plotting of PointCloud data 0312 , declaration!
	ros::Publisher last_pcl_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("icps/last_pcl_pub", 1);       //  for plotting of PointCloud data 0302 , declaration!
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_copied (new pcl::PointCloud<pcl::PointXYZ>);
	pcl_data_copied->header.frame_id = "map";
	pcl_data_copied->height = pcl_data_copied->width = 1;


	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_moving (new pcl::PointCloud<pcl::PointXYZ>);
	pcl_moving->header.frame_id = "map";
	pcl_moving->height = pcl_moving->width = 1;


	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_3 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_data_4 (new pcl::PointCloud<pcl::PointXYZ>);
	PCL_data_1->header.frame_id = "map";
	PCL_data_1->height = PCL_data_1->width = 1;
	PCL_data_2->header.frame_id = "map";
	PCL_data_2->height = PCL_data_2->width = 1;
	PCL_data_3->header.frame_id = "map";
	PCL_data_3->height = PCL_data_3->width = 1;
	PCL_data_4->header.frame_id = "map";
	PCL_data_4->height = PCL_data_4->width = 1;

	ros::Subscriber odom_sub;
	message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_ = NULL;
	message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_ = NULL;
	message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

	//ros::Publisher scan_polygon = nh.advertise<geometry_msgs::PolygonStamped> ("polygon", 1);
	//nh_ns.param("port", port, string("/dev/lms200"));
	//nh_ns.param("baud", baud, 38400);
	//nh_ns.param("inverted", inverted, false);
	nh_ns.param("angle", angle, 0);
	nh_ns.param("resolution", resolution, 0.0);
	nh_ns.param<std::string>("ip_add", ip_add, "192.168.123.2");
	nh_ns.param<std::string> ("frame_id", frame_id, "map");

	uint32_t range_values[SickLMS5xx::SICK_LMS_5XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t intensity_values[SickLMS5xx::SICK_LMS_5XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t n_range_values = 0;
	uint32_t n_intensity_values = 0;
	SickLMS5xx sick_lms(ip_add);
	double scale = 0;
	double angle_offset;
	uint32_t partial_scan_index;

	try {
		sick_lms.Initialize();
		sick_lms.SetSickScanFreqAndRes(SickLMS5xx::SICK_LMS_5XX_SCAN_FREQ_25,
				SickLMS5xx::SICK_LMS_5XX_SCAN_RES_17);
		/*sick_lms.SetSickScanFreqAndRes(SickLMS5xx::SICK_LMS_5XX_SCAN_FREQ_25,
		                               SickLMS5xx::SICK_LMS_5XX_SCAN_RES_25);*/
		//sick_lms.SetSickEchoFilter(SickLMS5xx::SICK_LMS_5XX_ECHO_FILTER_ALL_ECHOES);
		sick_lms.SetSickEchoFilter(SickLMS5xx::SICK_LMS_5XX_ECHO_FILTER_FIRST);

		// Scale is mm with the 5xx driver
		scale = 0.001;


	} catch(...) {
		ROS_ERROR("Initialize failed!");
		return 2;
	}
	try {

		Lidar_Object *Lidar;
		Lidar = new Lidar_Object(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
		//odom_sub = nh.subscribe("odom", 20 , &Lidar_Object::odomCallback , Lidar );

		ros::Time last_scan_time = ros::Time::now();
		while(ros::ok()) {
			//			angle_min = sick_lms.GetSickStartAngle() * M_PI / 180.0;
			//			angle_max = sick_lms.GetSickStopAngle()  * M_PI / 180.0;

			// Rotate the coordinate system so we get a range [-95;95] instead of [-5;185].
			// Avoids the need to manually rotate the laser scanner in the URDM_PI / 180.0F.
			angle_min = (sick_lms.GetSickStartAngle()-90.) * M_PI / 180.0;
			angle_max = (sick_lms.GetSickStopAngle()-90.)  * M_PI / 180.0;

			sick_lms.GetSickMeasurements(range_values, NULL, NULL, NULL, NULL,
					NULL, NULL, NULL, NULL, NULL,
					n_range_values);

			n_intensity_values = 0;

			ros::Time end_of_scan = ros::Time::now();
			ros::Time start = end_of_scan; // TODO - ros::Duration(scan_time / 2.0);
			ros::Duration diff = start - last_scan_time;
			last_scan_time = start;

			//			publish_scan( &Scan_msg , &Polygon_pub, &scan_pub, range_values, n_range_values, 			// FOR BOUNDING BOX !!
			//					intensity_values, n_intensity_values, scale, start,
			//					scan_time, inverted, angle_min, angle_max, frame_id);

			publish_scan( &Scan_msg , &scan_pub, range_values, n_range_values,										//version_3
					intensity_values, n_intensity_values, scale, start,
					scan_time, inverted, angle_min, angle_max, frame_id);				// the place where the laser_scan data written down

			ros::Duration(0.0025).sleep();
			Init_for_secondArr( &Scan_msg_Copied , range_values, n_range_values,
					start,	scan_time, inverted, angle_min, angle_max, frame_id );
			ros::Duration(0.0005).sleep();

			ros::spinOnce();															// the place odom data in queue pops up ! (Originally it was at the end of main func)

			Lidar->Vehicle_moving( &Scan_msg ,  &first_pcl_pub , pcl_data , &half_pcl_pub ,PCL_data_1 , PCL_data_2 , PCL_data_3 , PCL_data_4 , pcl_moving , &last_pcl_pub );

		}


	} catch(...) {
		ROS_ERROR("woah! error!");
		return 1;
	}
	try {
		sick_lms.Uninitialize();
	} catch(...) {
		ROS_ERROR("error during uninitialize");
		return 1;
	}
	ROS_INFO("success.\n");

	return 0;
}

