/*
 * ROSMsgHelper.h
 *
 *  Created on: Jan 25, 2018
 *      Author: retry
 */

#ifndef SRC_ICPS_INCLUDE_ROSHELPER_H_
#define SRC_ICPS_INCLUDE_ROSHELPER_H_

#include <string>

#include "ros/ros.h"
#include <ros/console.h>
#include "visualization_msgs/Marker.h"

#define ERRMSG(fmt,args...)	ROS_ERROR(fmt, ##args)
#define INFMSG(fmt,args...)	ROS_INFO(fmt, ##args)

#ifdef DEBUG
	#define DBGMSG(fmt,args...) ROS_DBG(fmt, ##args)
#else
	#define DBGMSG(fmt,args...) do {} while(0)
#endif

class ROSHelper{
public:
	static visualization_msgs::Marker createMarker(std::string _frameID, std::string _namespace, int _id, int _type, int _action,
			double _posX, double _posY, double _orienX, double _orienY, double _scaleX, double _scaleY,
			double _colorR, double _colorG, double _colorB, double _durationSec){
		visualization_msgs::Marker marker;

		marker.header.frame_id = _frameID;
		marker.header.stamp = ros::Time::now();
		marker.ns = _namespace;
		marker.id = _id;
		marker.type = _type;
		marker.action = _action;

	    marker.pose.position.x = _posX;
	    marker.pose.position.y = _posY;
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = _orienX;
	    marker.pose.orientation.y = _orienY;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    marker.scale.x = _scaleX;
	    marker.scale.y = _scaleY;
	    marker.scale.z = 1.0;

	    marker.color.r = _colorR;
	    marker.color.g = _colorG;
	    marker.color.b = _colorB;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration(_durationSec);
	    return marker;
	}
};


#endif /* SRC_ICPS_INCLUDE_ROSHELPER_H_ */
