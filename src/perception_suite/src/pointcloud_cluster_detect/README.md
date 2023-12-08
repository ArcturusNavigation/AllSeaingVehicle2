# pointcloud_cluster_detect
The purpose of this package is to detect objects in point cloud data regardless of the source (lidar or radar). Points are clustered and published as detected objects

## Process

## ROS API: pointcloud_euclidean_cluster_detect

### Subs
* `pc_topic` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))
Input point cloud from lidar or marine radar sensors

### Pubs

* `pc_topic/cluster_cloud` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Point Cloud of clustered objects
* `pc_topic/cluster_centroids` (auvlab_msgs/Centroids)  
Array of cluster centroids
* `pc_topic/detection/raw_cloud_clusters` (auvlab_msgs/CloudClusterArray)  
Array of cloud clusters
* `/nmea_to_send` ([nmea_msgs/Sentence](http://docs.ros.org/en/noetic/api/nmea_msgs/html/msg/Sentence.html))  
NMEA like message of cluster msgs to send to autonomy middleware
* `pc_topic/chull_markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))  
Marker array for visualization of clusters in RVIZ
* `pc_topic/text_markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))  
Marker array for visualization of cluster IDs in RVIZ

### ROS Parameters

* `pc_topic`: Input point cloud from lidar or marine radar sensor
* `nmea_msg_header`: Header to alert which type of sensor obstacle is derived from. Radar: PYROB, Lidar:PYLOB
* `filter_cloud`: Enable point cloud filtering around vehicle
* `remove_points`: Distance from center of point cloud to remove points to remove self-reflections
* `downsample_cloud`: Enable point cloud downsampling with voxel filter
* `leaf_size`: Leaf size applied to voxel filter for downsampling
* `cluster_size_min`: Minimum amount of points for cluster extraction
* `custer_size_max`: Maximum amount of points for cluster extraction
* `clustering_distance`: Euclidean cluster tolerance
* `output_frame`: ROS header frame id
* `cluster_seg_thresh`: Distance between tracked and current clusters clusters to be considered a match
* `drop_cluster_count`: Sweep count since cluster is last seen before dropping tracked cluster
* `viz`: Enable visualization of tracked obstacles as Markers in RVIZ

## cluster class
TODO: Document cluster class

## ROS API: pointcloud_sweep_aggregator
TODO: Document pointcloud_sweep_aggregator
