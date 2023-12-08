// 2021-03
// Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
// License: MIT
//

/************************************************************
// Usage
// roslaunch pointcloud_cluster_detect pointcloud_cluster_detect.launch pc_topic:="/broaand_radar/channel_0/pointcloud/revolution" filter_cloud:="true" remove_points:="3.0" downsampled_cloud:="true" leaf_size:="1.1" clustering_distance:="5.0" cluster_size_min:="20" cluster_size_max:="1000" cluster_seg_thresh:="5.0" nmea_msg_header:="PYROB"
************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/distances.h>  // euclideanDistance()
#include <pcl/filters/frustum_culling.h> // remove points not in camera fov

#include "cluster.h"

#include "auvlab_msgs/Centroids.h"
#include "auvlab_msgs/CloudCluster.h"
#include "auvlab_msgs/CloudClusterArray.h"

using namespace pcl;
using namespace std;

#define __APP_NAME__ "euclidean_cluster_detect"

typedef PointXYZI PointType;

ros::Publisher _pub_cluster_cloud;
ros::Publisher _centroid_pub;
ros::Publisher _pub_clusters_message;
ros::Publisher _pub_nmea_message;
ros::Publisher _pub_matched_clusters_msg;
ros::Publisher _pub_matched_clusters_cloud;
ros::Publisher _marker_array_pub;
ros::Publisher _text_marker_array_pub;
ros::Publisher _bb_marker_pub;
std_msgs::Header _sensor_header;
std_msgs::Time _current_time;

static string _output_frame;
static string _nmea_msg_header;

static bool _using_cloud = false;
static bool _filter_cloud = false;
static bool _downsampled_cloud = false;
static bool _use_multiple_thres = false;
static bool _viz = false;
static bool _filter_camera_view = false;

static double _camera_theta;
static int _camera_hfov;
static double _cull_min;
static double _cull_max;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;
static double _clustering_distance;
static double _cluster_seg_thresh;
static int _drop_cluster_count;
static double _drop_cluster_thresh;
static double _polygon_area_thresh;

static auvlab_msgs::CloudClusterArray prev_clusters;    // previous clusters not yet fully matched
static auvlab_msgs::CloudClusterArray matched_clusters;  // matching clusters from prev segments
static auvlab_msgs::CloudClusterArray tracking_clusters; // active clusters

static vector<ClusterPtr> tracked_clusters; // Active clusters
static int cluster_id = 0; 
static int sweep_count = 0;

/**
 * @brief publishes clusters in point cloud format 
 * @param [in] in_publisher          : ros publisher
 * @param [in] in_clusters           : array of clusters to publish point cloud clusters
 **/
void publishCloudClusters(const ros::Publisher *in_publisher,
			  const auvlab_msgs::CloudClusterArray &in_clusters,
			  const string &in_target_frame, const std_msgs::Header &in_header)
{
  // transorm target frame if needed
    
  in_publisher->publish(in_clusters);
  // publishDetectedObjects(in_clusters); Will eventually create a message to handle detections combined with classifier

}

/**
 * @brief publishes centroids of all tracked clusters
 * @param [in] in_publisher          : ros publisher
 * @param [in] in_centroids          : centroids to publish
 **/
void publishCentroids(const ros::Publisher *in_publisher, const auvlab_msgs::Centroids &in_centroids,
		      const string &in_target_frame, const std_msgs::Header &in_header)
{
  // transform target frame if needed

  in_publisher->publish(in_centroids);

}

/**
 * @brief publishes nmea messeages for each tracked cluster to report to MOOS
 * @param [in] in_publisher          : ros publisher
 * @param [in] in_clusters           : array of clusters to extract and publish nmea msg
 **/
void publishNMEA(const ros::Publisher *in_publisher,
		 const auvlab_msgs::CloudClusterArray &in_clusters)
{
  for(auto cluster : in_clusters.clusters)
    {
      in_publisher->publish(cluster.nmea_msg);
    }
}

/**
 * @brief clusters current point cloud using Euclidean cluster extraction
 * @param [in] in_cloud_ptr             :point cloud to cluster
 * @param [in] out_cloud_ptr            :
 * @param [in] in_out_centroids         :
 * @param [in] in_max_cluster_distance  :Euclidean cluster tolerance
 * @param [in] nmea_msg_header          :nmea message header to track which type of sensor the clustered object has been created from. Radar: PYROB, Lidar: PYLOB
 *
 * @return vector of Cluster objects determined by the cluster extraction algorithm
 */
vector<ClusterPtr> clusterCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,
				PointCloud<PointType>::Ptr out_cloud_ptr,
				auvlab_msgs::Centroids &in_out_centroids,
				double in_max_cluster_distance = 0.5,
				std::string nmea_msg_header = "$PYXOB")
{
  //cout << "in_cloud_ptr->points.size() " << in_cloud_ptr->points.size() << endl;

  
  search::KdTree<PointType>::Ptr tree(new search::KdTree<PointType>);

  // create 2d pointcloud
  PointCloud<PointType>::Ptr cloud_2d(new PointCloud<PointType>);
  copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // flatten cloud
  for (size_t i=0; i<cloud_2d->points.size(); i++)
    {
      cloud_2d->points[i].z = 0;
    }

    
  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  // extract clusters from point cloud and save indices in cluster_indices
  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(in_max_cluster_distance);
  ec.setMinClusterSize(_cluster_size_min);
  ec.setMaxClusterSize(_cluster_size_max);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);

  //vector<PointCloud<PointType>::Ptr > clusters;

  // Cluster Points
  //unsigned int k = 0;
  vector<ClusterPtr> clusters;

  //cout << "cluster_indices.size() " << cluster_indices.size() << endl;

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      ClusterPtr cluster(new Cluster());
      cluster->SetCloud(in_cloud_ptr, it->indices, _sensor_header, cluster_id, "",
			nmea_msg_header, _current_time);
      clusters.push_back(cluster);

      cluster_id++;
    }

  return (clusters);
  

}

/** @brief matches tracked clusters to current time step clusters. 
 * 
 * If cluster is complete pushes full cluster to auvlab_msgs::CloudClusterArray matched_clusters
 *
 * @param[in] in_clusters      :current sensor sweep clusters (all_clusters) 
 * @param[in] in_out_clusters  :tracked clusters (tracked_clusters)
 */

void matchClusters(vector<ClusterPtr> &in_clusters,
		    vector<ClusterPtr> &in_out_clusters)
{
  vector<ClusterPtr> track_clusters;  // vector of Cluster object type returned from clusterCloud

  // check in_out_clusters for aged out clusters
  for(int i=0; i<in_out_clusters.size(); i++)
    {
      float last_seen;
      std_msgs::Time cluster_last_seen = in_out_clusters[i]->GetLastSeen();
      last_seen = _current_time.data.toSec() - cluster_last_seen.data.toSec();
      
      if (last_seen >= _drop_cluster_thresh)
	{
	  //cout << "del in_out_clusters[i]->GetID(): " << in_out_clusters[i]->GetID() << endl;
	  if(i>0)
	    {
	      int del = i - 1;
	      in_out_clusters.erase(in_out_clusters.begin()+del);
	    }
	  else
	    in_out_clusters.erase(in_out_clusters.begin());
	}
    }
  
  // find closest clusters and combine if below threshold
  for(int i=0; i<in_clusters.size(); i++)
    {
      // Check if Area is to large to be a valid clustered object
      if (in_clusters[i]->GetPolygonArea() > _polygon_area_thresh)
	{
	  // prune object
	  in_clusters.erase(in_clusters.begin()+i);
	  continue;
	}
      float dist = _cluster_seg_thresh;
      int best_match = -1;
      for(int j=0; j<in_out_clusters.size(); j++)
	{
	  float this_dist = euclideanDistance(in_out_clusters[j]->GetCentroidPoint(),
					      in_clusters[i]->GetCentroidPoint());
	  if(this_dist < dist)
	    {
	      best_match = j;
	      dist = this_dist; // update threshold
	    }
	} // end for
      
      if(best_match >=0)
	{
	  /*
	  cout << "best_match " << best_match << endl;
	  cout << "in_out_clusters[best_match]->GetID() "
	       << in_out_clusters[best_match]->GetID() << endl;
	  cout << "in_out_clusters[best_match]->GetLastSeen() "
		<< in_out_clusters[best_match]->GetLastSeen() << endl;
	  cout << "before match: in_clusters[i]->GetID() " << in_clusters[i]->GetID() << endl;
	  cout << "before match: in_clusters[i]->GetLastSeen() "
	       << in_clusters[i]->GetLastSeen() << endl;
	  */
	  // change cluster new cluster id to matched cluster id
	  in_clusters[i]->SetID(in_out_clusters[best_match]->GetID());
	  track_clusters.push_back(in_clusters[i]);
	  //cout << "after match: in_clusters[i]->GetID() " << in_clusters[i]->GetID() << endl;
	  //cout << "after match: in_clusters[i]->GetLastSeen() "
	  //     << in_clusters[i]->GetLastSeen() << endl;
	}
      else
	{
	  //in_out_clusters.push_back(in_clusters[i]); // 
	  track_clusters.push_back(in_clusters[i]);
	}
    } // end for

  // if no tracked clusters copy current clusters into tracked
  if(in_out_clusters.size() < 1)
    {
      //cout << "Set current clusters to tracked clusters" << endl;
      in_out_clusters = in_clusters;
      //cout << "new tracked clusters size: " << in_out_clusters.size() << endl;
    }

  else
    {
      // clear in_out_clusters
      //in_out_clusters.clear();

      //cout << "before adding track_clusters to in_out_clusters: " << in_out_clusters.size() << endl;
      // Push all new tracked clusters into in_out_clusters
      for(int i=0; i<track_clusters.size(); i++)
	in_out_clusters.push_back(track_clusters[i]);

      //cout << "after adding track_clusters to in_out_clusters: " << in_out_clusters.size() << endl;   
    }
}

/**
 * @brief point cloud segmentation
 * @param[in] in_cloud_ptr     :input point cloud
 * @param[in] out_cloud_ptr    :segmented and clustered point cloud
 * @param[in] in_out_centroids :auvlab_msgs::Centroids, custom message to track centroid of each clustered object 
 * @param[in] in_out_clusters  :auvlab_msgs::CloudClusterArray, custom message to track objects individually
 * @param[in] nmea_msg_header  :nmea message header to track which type of sensor the clustered object has been created from. Radar: PYROB, Lidar: PYLOB
 */

void segmentCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,
		  PointCloud<PointType>::Ptr out_cloud_ptr,
		  auvlab_msgs::Centroids &in_out_centroids,
		  auvlab_msgs::CloudClusterArray &in_out_clusters,
		  std::string nmea_msg_header)
{
  // Cluster the pointcloud by the distance of the points using different thresholds
  // This helps with clustering at larger distances. See adaptive clustering for more info
  // thresholds are based off empirical evidence, further study required for larger distances

  // 0 => 0-20m   d=0.5
  // 1 => 20-50   d=1.0
  // 2 => 50-100  d=1.5
  // 3 => 100-300 d=2.0
  // 4 => >300    d=3.0

  vector<ClusterPtr> all_clusters;  // vector of Cluster object type returned from clusterCloud

  if(!_use_multiple_thres)
    {
      PointCloud<PointType>::Ptr cloud_ptr(new PointCloud<PointType>);

      for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
	{
	  PointType current_point;
	  current_point.x = in_cloud_ptr->points[i].x;
	  current_point.y = in_cloud_ptr->points[i].y;
	  current_point.z = in_cloud_ptr->points[i].z;

	  cloud_ptr->points.push_back(current_point);
	}

      all_clusters = clusterCloud(cloud_ptr, out_cloud_ptr, in_out_centroids,
				  _clustering_distance, nmea_msg_header);
      
    }

  // Push raw clusters (all_clusters) to CloudClusterArray and publish
  auvlab_msgs::CloudClusterArray raw_cloud_clusters;
  for (unsigned int i=0; i<all_clusters.size(); i++)
    {
      if (all_clusters[i]->IsValid())
	{
	  auvlab_msgs::CloudCluster raw_cloud_cluster;
	  all_clusters[i]->ToROSMessage(_sensor_header, raw_cloud_cluster);
	  raw_cloud_clusters.clusters.push_back(raw_cloud_cluster);
	}
    }
  publishCloudClusters(&_pub_clusters_message, raw_cloud_clusters, _output_frame, _sensor_header);
  
  
  // current sensor clusters (all_clusters), tracked_clusters
  matchClusters(all_clusters, tracked_clusters);
  
  // final pointcloud to be published  // SHould be for the current sweep
  for (unsigned int i=0; i<all_clusters.size(); i++)
    //for (unsigned int i=0; i<tracked_clusters.size(); i++)
    {
      *out_cloud_ptr += *all_clusters[i]->GetCloud();
      //*out_cloud_ptr += *tracked_clusters[i]->GetCloud();

      PointType center_point = all_clusters[i]->GetCentroidPoint();
      //PointType center_point = tracked_clusters[i]->GetCentroidPoint();
      geometry_msgs::Point centroid;
      centroid.x = center_point.x;
      centroid.y = center_point.y;
      centroid.z = center_point.z;

      // Push matched clusters to CloudClusterArray
      if (all_clusters[i]->IsValid())
	//if (tracked_clusters[i]->IsValid())
	{
	  in_out_centroids.points.push_back(centroid);

	  auvlab_msgs::CloudCluster cloud_cluster;
	  all_clusters[i]->ToROSMessage(_sensor_header, cloud_cluster);
	  //tracked_clusters[i]->ToROSMessage(_sensor_header, cloud_cluster);
	  in_out_clusters.clusters.push_back(cloud_cluster);
	  //cout << "cloud cluster id: " << cloud_cluster.id << endl;
	}
    }
}

/**
 * @brief point cloud publisher
 * @param [in] in_pubisher                 :ros publisher
 * @param [in] in_cloud_to_publish_ptr     :point cloud to publish 
 */
void publishCloud(const ros::Publisher *in_publisher,
		  const PointCloud<PointType>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _sensor_header;
  in_publisher->publish(cloud_msg);
}

/**
 * @brief filter points around vessel to remove self reflections
 * @param [in] in_cloud_ptr     :input point cloud to filter
 * @param [in] out_cloud_ptr    :output point cloud to store filtered cloud
 * @param [in] in_distance      :min distance allowed to keep point
 */
void filterCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,
		 PointCloud<PointType>::Ptr out_cloud_ptr,
		 const double min_distance,
		 const double max_distance
		 )
{
  out_cloud_ptr->points.clear();
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
      float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) +
				   pow(in_cloud_ptr->points[i].y, 2));
      if (origin_distance > min_distance && origin_distance < max_distance)
	{
	  out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
	}
    }
}

/**
 * @brief filter out points that are not in the view of the camera. Try using Frustum Culling if it is supported by ROS
 * @param [in] in_cloud_ptr     :input pointcloud to filter
 * @param [in] out_cloud_ptr    :output pointcloud to store filtered cloud
 * @param [in] fov              :horizontal fov of the camera
 * @param [in] theta            :pose of the camera w.r.t the camera origin
   //                   Frustum and the vectors a, b, c and d. T is the position of the camera
   //                        http://docs.ros.org/en/hydro/api/pcl/html/frustum__culling_8hpp_source.html
   //                             _________
   //                           /|       . |
   //                       d  / |   c .   |
   //                         /  | __._____| 
   //                        /  /  .      .
   //                 a <---/-/  .    .
   //                      / / .   .  b
   //                     /   .
   //                     . 
   //                   T
   //
 */
void filterCameraView(const PointCloud<PointType>::Ptr in_cloud_ptr,
		      PointCloud<PointType>::Ptr out_cloud_ptr,
		      const int fov,
		      const double theta)
{
  float half_fov = float (fov * M_PI / 180) / 2;
  float camera_theta = float (theta * M_PI / 180);
  out_cloud_ptr->points.clear();
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
      float current_theta = atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x);
      //cout << "current theta: " << current_theta << ", camrera theta: " << camera_theta << ", camera theta-half_fov: " <<
      //camera_theta-half_fov  << ", camera theta + half_fov: " << camera_theta + half_fov << endl;
      if (current_theta > camera_theta - half_fov && current_theta < camera_theta + half_fov)
	{
	  out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
	}
    }
  
}

/**
 * @brief voxel filter to downsample point cloud
 * @param [in] in_cloud_ptr     :input point cloud to downsample
 * @param [in] out_cloud_ptr    :output point cloud to store downsampled cloud
 * @param [in] in_leaf_size     :leaf size to downsample cloud 
 */
void downsampledCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,
		      PointCloud<PointType>::Ptr out_cloud_ptr, float in_leaf_size = 0.2)
{
  VoxelGrid<PointType> vg;
  vg.setInputCloud(in_cloud_ptr);
  vg.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);
  vg.filter(*out_cloud_ptr);
}

/**
 * @brief function to add convex hull markers and obstacle id for visualization in RVIZ
 * @param [in] in_cluster_array     :current tracked cluster array
 */
void markers(auvlab_msgs::CloudClusterArray in_cluster_array)
{
  visualization_msgs::MarkerArray markers_array;
  visualization_msgs::MarkerArray text_markers_array;
  visualization_msgs::MarkerArray bb_markers_array;

  //cout << "in_cluster_array.clusters.size() :" << in_cluster_array.clusters.size() << endl;

  for(int i=0; i < in_cluster_array.clusters.size(); i++)
    {
      //cout << "in_cluster_array.clusters[i].id: " << in_cluster_array.clusters[i].id << endl;
      visualization_msgs::Marker marker;
      //marker.header.stamp = ros::Time::now();
      marker.header.stamp = _sensor_header.stamp;
      marker.header.frame_id = _sensor_header.frame_id;
      marker.header.seq = _sensor_header.seq;
      marker.ns = "clustering";
      marker.id = in_cluster_array.clusters[i].id; // 
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 1.0; // point width, scale.y = point height
      marker.lifetime = ros::Duration(1.3);
      //marker.text = to_string(i);
      marker.scale.z = 1.0;

      /*
      visualization_msgs::Marker bb_marker;
      bb_marker.type = visualization_msgs::Marker visualization_msgs::Marker::LINE_STRIP;
      bb_marker.stamp = _sensor_header.stamp;
      bb_marker.frame_id = _sensor_header.frame_id;
      bb_marker.seq = _sensor_header.seq;
      bb_marker.ns = "bounding_box";
      bb_marker.id = in_cluser_array.clusters[i].id;
      bb_marker.color.a = 0.5;
      bb_marker.color.g = 1.0;
      bb_marker.scale.x = 1.0;
      bb_marker.scale.z = 1.0;
      //bb_marker.lifetime = ros::Duration(1.3);
      */

      visualization_msgs::Marker text_marker;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.header = _sensor_header;
      text_marker.ns = "text_clustering";
      text_marker.id = in_cluster_array.clusters[i].id; 
      text_marker.scale.z = 10.0;  // change for text scale in RVIZ
      text_marker.color.a = 1.0;
      text_marker.color.g = 1.0;
      text_marker.lifetime = ros::Duration(1.3);
      text_marker.text = to_string(in_cluster_array.clusters[i].id);
      
      text_marker.pose.position.x = in_cluster_array.clusters[i].max_point.point.x + 3;
      text_marker.pose.position.y = in_cluster_array.clusters[i].centroid_point.point.y;
      text_marker.pose.position.z = in_cluster_array.clusters[i].centroid_point.point.z;
      text_marker.pose.orientation.x = 0.0;
      text_marker.pose.orientation.y = 0.0;
      text_marker.pose.orientation.z = 0.0;
      text_marker.pose.orientation.w = 1.0;
      text_markers_array.markers.push_back(text_marker);
      
      for(int j=0; j<in_cluster_array.clusters[i].convex_hull.polygon.points.size(); j++)
	{
	  geometry_msgs::Point p;
	  p.x = in_cluster_array.clusters[i].convex_hull.polygon.points[j].x;
	  p.y = in_cluster_array.clusters[i].convex_hull.polygon.points[j].y;
	  p.z = in_cluster_array.clusters[i].convex_hull.polygon.points[j].z;
	  marker.points.push_back(p);
	}
      geometry_msgs::Point p;
      p.x = in_cluster_array.clusters[i].convex_hull.polygon.points[0].x;
      p.y = in_cluster_array.clusters[i].convex_hull.polygon.points[0].y;
      p.z = in_cluster_array.clusters[i].convex_hull.polygon.points[0].z;

      marker.points.push_back(p);
      markers_array.markers.push_back(marker);
       
    }
  _marker_array_pub.publish(markers_array);
  _text_marker_array_pub.publish(text_markers_array);
  //_bb_marker_pub.publish(bb_markers_array);
}

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud)
{
  if (!_using_cloud)
    {
      _using_cloud = true;
      _current_time.data = in_cloud->header.stamp;  //ros::Time::now();  
      
      PointCloud<PointType>::Ptr current_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr filtered_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr downsampled_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr clustered_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr camera_cloud_ptr(new PointCloud<PointType>);

      auvlab_msgs::Centroids centroids;   // current segment centroids
      auvlab_msgs::CloudClusterArray cloud_clusters;    // current segments clusters
      
      fromROSMsg(*in_cloud, *current_cloud_ptr);
      _sensor_header = in_cloud->header;


      if (_filter_cloud)
	{
	  filterCloud(current_cloud_ptr, filtered_cloud_ptr, _cull_min, _cull_max);
	}
      else
	{
	  filtered_cloud_ptr = current_cloud_ptr;
	}

      if (_downsampled_cloud)
	{
	  downsampledCloud(filtered_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
	}
      else
	{
	  downsampled_cloud_ptr = filtered_cloud_ptr;
	}

      if (_filter_camera_view)
	{
	  filterCameraView(downsampled_cloud_ptr, camera_cloud_ptr, _camera_hfov, _camera_theta);
	}
      else
	{
	  camera_cloud_ptr = downsampled_cloud_ptr;
	}	  


      //segmentCloud(downsampled_cloud_ptr, clustered_cloud_ptr, centroids, cloud_clusters, _nmea_msg_header);
      segmentCloud(camera_cloud_ptr, clustered_cloud_ptr, centroids, cloud_clusters,
		   _nmea_msg_header);

      sweep_count++;

      // cluster_cloud publisher (raw cloud publisher)
      publishCloud(&_pub_cluster_cloud, clustered_cloud_ptr);

      centroids.header = _sensor_header;
      publishCentroids(&_centroid_pub, centroids, _output_frame, _sensor_header);

      // Cloud clusters message publisher (matched clusters)
      cloud_clusters.header = _sensor_header;
      publishCloudClusters(&_pub_matched_clusters_msg, cloud_clusters, _output_frame, _sensor_header);
      publishNMEA(&_pub_nmea_message, cloud_clusters); // 

      // Visualization
      if (_viz)
	{
	  markers(cloud_clusters);
	}
      
      _using_cloud = false;
    }
}


int main(int argc, char **argv)
{
  // Init ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string pc_topic, sensor, app;
  
  if(private_nh.getParam("pc_topic", pc_topic))
    {
      ROS_INFO("euclidean_cluster > Setting pc_topic to %s", pc_topic.c_str());
    }
  else
    {
      ROS_INFO("euclidean_cluster > No pointcloud received, defaulting to velodyne_points, use _pc_topic = /topic to set your points topic");
      pc_topic = "/velodyne_points";
    }
  private_nh.getParam("sensor", sensor);
  
  _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>(pc_topic + "/cluster_cloud", 100);
  _centroid_pub = nh.advertise<auvlab_msgs::Centroids>(pc_topic + "/cluster_centroids", 1);
  _pub_clusters_message =
    nh.advertise<auvlab_msgs::CloudClusterArray>(pc_topic + "/detection/raw_cloud_clusters", 1);
  _pub_matched_clusters_msg =
    nh.advertise<auvlab_msgs::CloudClusterArray>(pc_topic + "/detection/matched_cloud_clusters", 1);
  //_pub_matched_clusters_cloud =
  //  nh.advertise<sensor_msgs::PointCloud2>(pc_topic + "/matched_clusters_cloud", 100);
  _pub_nmea_message = nh.advertise<nmea_msgs::Sentence>("/nmea_to_send", 100);
  _marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(pc_topic + "/chull_markers", 100);
  _text_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(pc_topic + "/text_markers", 100);
  //_bb_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(pc_topic + "/bb_markers", 100);
  
  _using_cloud = false;

  

  // Init tuning parameters
  private_nh.param("filter_camera_view", _filter_camera_view, false);
  ROS_INFO("[%s] filter_camera_view: %d", __APP_NAME__, _filter_camera_view);
  private_nh.param("camera_hfov", _camera_hfov, 80);
  ROS_INFO("[%s] camera_hfov: %d", __APP_NAME__, _camera_hfov);
  private_nh.param("camera_theta", _camera_theta, 0.0);
  ROS_INFO("[%s] camera_theta: %f", __APP_NAME__, _camera_theta);
  private_nh.param("filter_cloud", _filter_cloud, false);
  ROS_INFO("[%s] filter_cloud: %d", __APP_NAME__, _filter_cloud);
  private_nh.param("cull_min", _cull_min, 0.0);
  ROS_INFO("[%s] cull_min: %f", __APP_NAME__, _cull_min);
  private_nh.param("cull_max", _cull_max, 0.0);
  ROS_INFO("[%s] cull_max: %f", __APP_NAME__, _cull_max);
  private_nh.param("downsampled_cloud", _downsampled_cloud, false);
  ROS_INFO("[%s] downsampled_cloud %d", __APP_NAME__, _downsampled_cloud);
  private_nh.param("leaf_size", _leaf_size, 0.0);
  ROS_INFO("[%s] leaf_size %f", __APP_NAME__, _leaf_size);
  private_nh.param("cluster_size_min", _cluster_size_min, 20);
  ROS_INFO("[%s] cluster_size_min %d", __APP_NAME__, _cluster_size_min);
  private_nh.param("cluster_size_max", _cluster_size_max, 100000);
  ROS_INFO("[%s] cluster_size_max %d", __APP_NAME__, _cluster_size_max);
  private_nh.param("clustering_distance", _clustering_distance, 0.75);
  ROS_INFO("[%s] clustering_distance %f", __APP_NAME__, _clustering_distance);
  private_nh.param("use_multiple_thres", _use_multiple_thres, false);
  ROS_INFO("[%s] use_multiple_thres %d", __APP_NAME__, _use_multiple_thres);
  private_nh.param<string>("output_frame", _output_frame, "velodyne");
  ROS_INFO("[%s] output_frame: %s", __APP_NAME__, _output_frame.c_str());
  private_nh.param("cluster_seg_thresh", _cluster_seg_thresh, 1.0);
  ROS_INFO("[%s] cluster_seg_thresh: %f", __APP_NAME__, _cluster_seg_thresh);
  private_nh.param("drop_cluster_count", _drop_cluster_count, 5);
  ROS_INFO("[%s] drop_cluster_count: %d", __APP_NAME__, _drop_cluster_count);
  private_nh.param("drop_cluster_thresh", _drop_cluster_thresh, 1.0);
  ROS_INFO("[%s] %s drop_cluster_thresh: %f", __APP_NAME__, sensor.c_str(), _drop_cluster_thresh);
  private_nh.param("polygon_area_thresh", _polygon_area_thresh, 100000.0);
  ROS_INFO("[%s] polygon_area_thresh: %f", __APP_NAME__, _polygon_area_thresh);
  private_nh.param<string>("nmea_msg_header", _nmea_msg_header, "PYXOB");
  _nmea_msg_header = "$" + _nmea_msg_header;
  ROS_INFO("[%s] nmea_msg_header: %s", __APP_NAME__, _nmea_msg_header.c_str());
  private_nh.param("viz", _viz, false);
  ROS_INFO("[%s] viz: %d", __APP_NAME__, _viz);
  
  // Subscribe to the input point cloud 'pc_topic'
  ros::Subscriber sub = nh.subscribe(pc_topic, 100, pc_callback);

  // Spin
  ros::spin();
}
