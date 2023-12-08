// 2021-04
// Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
// License: MIT
//

/* \brief pointcloud sweep aggregator node to produce a single pointcloud per radar revolution */


#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <auvlab_msgs/RadarSpoke.h>
#include <auvlab_msgs/RadarSegment.h>
#include <iostream>

#define SPOKES_IN_SEGMENT 32
#define PIXELS_IN_SPOKE 512

using namespace std;
using namespace pcl;

// globals
float check_angle = 0.0;
typedef PointXYZI PointType;

PointCloud<PointType>::Ptr cloud_ptr(new PointCloud<PointType>);

ros::Publisher _pub_cloud_aggregate;

void segments_to_pcl_cb(auvlab_msgs::RadarSegment segment)
{
  
  for(int spoke=0; spoke<segment.spokes.size(); spoke++)
    {
      
      float add = (segment.spokes[spoke].angle - check_angle);
      if (add < 0.0)
	add = 360 - add;
      check_angle += add;
      if(check_angle < 360.0)
	{
	  float angle = segment.spokes[spoke].angle - 270; // correct for orientation forward
	  float max_range = segment.spokes[spoke].max_range;

	  for(int bin=0; bin<PIXELS_IN_SPOKE; bin++)
	    {
	      PointType p;
	      p.x = max_range * bin/(PIXELS_IN_SPOKE-1) * sin(angle * M_PI/180.0);
	      p.y = max_range * bin/(PIXELS_IN_SPOKE-1) * cos(angle * M_PI/180.0);
	      p.z = 0.0;
	      p.intensity = segment.spokes[spoke].data[bin];
	      cloud_ptr->push_back(p);
	    }
	} // end if check for 0.0 angle
      else
	{
	  check_angle = 0.0; // reset angle check
	  // filter data to remove zero intensity points
	  PointCloud<PointType>::Ptr cloud_filtered_ptr(new PointCloud<PointType>);
	  PassThrough<PointType> pass;
	  pass.setInputCloud(cloud_ptr);
	  pass.setFilterFieldName("intensity");
	  pass.setFilterLimits(1.0, FLT_MAX);
	  pass.filter(*cloud_filtered_ptr);
	  
	  // convert to ros msg
	  sensor_msgs::PointCloud2 cloud_msg;
	  toROSMsg(*cloud_filtered_ptr, cloud_msg);
	  cloud_msg.header.stamp = segment.header.stamp;
	  cloud_msg.header.frame_id = "velodyne";  // TODO: Change to rader with parameter 
	  	  
	  // publish radar sweep
	  _pub_cloud_aggregate.publish(cloud_msg);
	  
	  // clear cloud_ptr
	  cloud_ptr->clear();
	}
    }
  
} // end segments_to_pcl_cb

int main(int argc, char **argv)
{
  // init ros
  ros::init (argc, argv, "aggregator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  _pub_cloud_aggregate = nh.advertise<sensor_msgs::PointCloud2>("/broadband_radar/channel_0/pointcloud/revolution", 1000);
  
  // init parameters
  string _segment;
  if ( private_nh.getParam("segment_topic", _segment))
    {
      ROS_INFO("segment_topic: %s", _segment.c_str());
    }
  else
    {
      ROS_INFO("no segment topic set, defaulting to /broadband_radar/channel_0/segment");
      _segment = "/broadband_radar/channel_0/segment";
    }

  // Subscribe to the segment topic 'segment'
  ros::Subscriber sub = nh.subscribe(_segment, 1000, segments_to_pcl_cb);

  // Spin
  ros::spin();

}
