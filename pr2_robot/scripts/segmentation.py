#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
	
# TODO: Convert ROS msg to PCL data
	cloud=ros_to_pcl(pcl_msg)
# TODO: Voxel Grid Downsampling
	#create a Voxel filter  object forourinput point cloud
	vox= cloud.make_voxel_grid_filter()
	##choose a voxel (also known as lead) size 
	#Note : this (1) is a poor choice of leaf size
	# Experiment and find the appropriate size !
	LEAF_SIZE= 0.01
	# set the voxel (or leaf) size
	vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
	#Call the filter function to obtain the resultant downsample point cloud
	cloud_filtered = vox.filter()
	filename = 'voxel_downsampled.pcd'
	pcl.save(cloud_filtered, filename)
# TODO: PassThrough Filter
	#Create a PassThrough filter object
	passthrough = cloud_filtered.make_passthrough_filter()
	# Assign axis and range to the passthrough filter object.
	filter_axis='z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min=0.76 #0.6
	axis_max=1.8 #1.1
	passthrough.set_filter_limits(axis_min,axis_max)
	# Finally use the filter function to obtain the resultant point cloud
	cloud_filtered = passthrough.filter()
	filename='pass_through_filtered.pcd'
	pcl.save(cloud_filtered, filename)
# TODO: RANSAC Plane Segmentation
	#create the segmentation object
	seg = cloud_filtered.make_segmenter()
	
	#Set the model you wish to fit 
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	# Max distance for a point to be considered fitting the model
	# Experiment with different values for max_distance 
	# for segmenting the table
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)
	# Call the segment function to obtain set of inlier indices and model coefficients
	inliers, coefficients = seg.segment()
# TODO: Extract inliers and outliers
	# Extract inliers
	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	filename = 'cloud_table.pcd'
	pcl.save(extracted_inliers, filename)
	# Save pcd for table
	# pcl.save(cloud, filename)
	
	# Extract outliers
	extracted_outliers = cloud_filtered.extract(inliers, negative=True)
	filename = 'cloud_objects.pcd'
	pcl.save(extracted_outliers, filename)
# TODO: Convert PCL data to ROS msg
	ros_cloud_objects = pcl_to_ros(extracted_outliers)
	ros_cloud_table = pcl_to_ros(extracted_inliers)
# TODO: Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(extracted_outliers)  # Apply function to convert XYZRGB to XYZ
	tree = white_cloud.make_kdtree()
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.	
	ec.set_ClusterTolerance(0.05)    #0.03
	ec.set_MinClusterSize(6)         #6
	ec.set_MaxClusterSize(1300)      #2000
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()
##cluster_indices now contains a list of indices for each cluster (a list of lists). 
#In the next step, you'll create a new point cloud to visualize the clusters by assigning a color to each of them.

# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
#Assign a color corresponding to each segmented object in scene
	cluster_color = get_color_list(len(cluster_indices))
	color_cluster_point_list = []
	for j, indices in enumerate(cluster_indices):
    		for i, indice in enumerate(indices):
        		color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])
	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
# TODO: Convert PCL data to ROS messages
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)
# TODO: Publish ROS msg
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':
#1 TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)
#2 TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
#Create Publishers. Here you're creating two new publishers 
##to publish the point cloud data for the table and the objects on the table to topics called pcl_table and pcl_objects, respectively.
#3 TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster",PointCloud2, queue_size=1)

    # Initialize color_list
	get_color_list.color_list = []

#4 TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()

