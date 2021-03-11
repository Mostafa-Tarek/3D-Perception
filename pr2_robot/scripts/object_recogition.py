#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
        
def voxel_filter(cloud_fliter):
	vox= cloud_fliter.make_voxel_grid_filter()
	##choose a voxel (also known as lead) size 
	#Note : this (1) is a poor choice of leaf size
	# Experiment and find the appropriate size !
	LEAF_SIZE= 0.01
	# set the voxel (or leaf) size
	vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
	#Call the filter function to obtain the resultant downsample point cloud
	cloud_filtered_v = vox.filter()
	filename = 'voxel_downsampled.pcd'
	pcl.save(cloud_filtered_v, filename)
	return cloud_filtered_v
def passthrough_filter(cloud_filtered_v,filter_axis,axis_min,axis_max):
	passthrough = cloud_filtered_v.make_passthrough_filter()
	# Assign axis and range to the passthrough filter object.
	passthrough.set_filter_field_name(filter_axis)
	passthrough.set_filter_limits(axis_min,axis_max)
	# Finally use the filter function to obtain the resultant point cloud
	cloud_filtered_pass = passthrough.filter()
	filename='pass_through_filtered.pcd'
	pcl.save(cloud_filtered_pass, filename)
	return cloud_filtered_pass
def RANSCAN_filter(cloud_filtered_pass):
	#create the segmentation object
	seg = cloud_filtered_pass.make_segmenter()
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
	extracted_inliers = cloud_filtered_pass.extract(inliers, negative=False)
	filename = 'cloud_table.pcd'
	pcl.save(extracted_inliers, filename)
	# Save pcd for table
	# pcl.save(cloud, filename)
	# Extract outliers
	extracted_outliers = cloud_filtered_pass.extract(inliers, negative=True)
	filename = 'cloud_objects.pcd'
	pcl.save(extracted_outliers, filename)
	return extracted_inliers,extracted_outliers
def Euclidean_Clustering(extracted_inliers,extracted_outliers):
	white_cloud = XYZRGB_to_XYZ(extracted_outliers)  # Apply function to convert XYZRGB to XYZ
	tree = white_cloud.make_kdtree()
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.	
	ec.set_ClusterTolerance(0.03)    #0.03
	ec.set_MinClusterSize(115)        #130 for test1 and 60 for test3
	ec.set_MaxClusterSize(3250)   #3000 for test1,2
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()
	return cluster_indices,white_cloud
def Cluster_Mask_Point(cluster_indices,white_cloud):
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
	return cluster_cloud
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
	
# TODO: Convert ROS msg to PCL data
	cloud=ros_to_pcl(pcl_msg)

# TODO: Voxel Grid Downsampling
	#create a Voxel filter  object forourinput point cloud
	cloud_filtered_v=voxel_filter(cloud)
# TODO: PassThrough Filter
	#Create a PassThrough filter object
	#cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'x',0.6,1.1)
	cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'y',-0.4,0.47)
	cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'z',0.6,0.91)
# TODO: RANSAC Plane Segmentation
	extracted_inliers,extracted_outliers=RANSCAN_filter(cloud_filtered_pass)
# TODO: Convert PCL data to ROS msg
	ros_cloud_objects = pcl_to_ros(extracted_outliers)
	ros_cloud_table = pcl_to_ros(extracted_inliers)
# TODO: Euclidean Clustering
	cluster_indices,white_cloud=Euclidean_Clustering(extracted_inliers,extracted_outliers)
##cluster_indices now contains a list of indices for each cluster (a list of lists). 
#In the next step, you'll create a new point cloud to visualize the clusters by assigning a color to each of them.
# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
#Assign a color corresponding to each segmented object in scene
	cluster_cloud=Cluster_Mask_Point(cluster_indices,white_cloud)
# TODO: Convert PCL data to ROS messages
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)
# TODO: Publish ROS msg
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)
# Exercise-3 TODOs: 
    # Classify the clusters! (loop through each detected cluster one at a time)
    # Classify the clusters!
    	detected_objects_labels = []
    	detected_objects = []
    	for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        	pcl_cluster = extracted_outliers.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
		ros_cluster=pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
	 # Extract histogram features
            	chists = compute_color_histograms(ros_cluster, using_hsv=True)#False
            	normals = get_normals(ros_cluster)
            	nhists = compute_normal_histograms(normals)
            	feature = np.concatenate((chists, nhists))
            	#labeled_features.append([feature, model_name])
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        	prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        	label = encoder.inverse_transform(prediction)[0]
        	detected_objects_labels.append(label)
        # Publish a label into RViz
        	label_pos = list(white_cloud[pts_list[0]])
        	label_pos[2] += .4
        	object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        	do = DetectedObject()
        	do.label = label
        	do.cloud = ros_cluster
        	detected_objects.append(do)
    	rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

if __name__ == '__main__':
#1 TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)
#2 TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
#Create Publishers. Here you're creating two new publishers 
##to publish the point cloud data for the table and the objects on the table to topics called pcl_table and pcl_objects, respectively.
#3 TODO: Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster",PointCloud2, queue_size=1)
	object_markers_pub = rospy.Publisher("/object_markers",Marker, queue_size=1)
	detected_objects_pub= rospy.Publisher("/detected_objects",DetectedObjectsArray, queue_size=1)
    # TODO: Load Model From disk
    # Load Model From disk
    	model = pickle.load(open('model.sav', 'rb'))
    	clf = model['classifier']
    	encoder = LabelEncoder()
    	encoder.classes_ = model['classes']
    	scaler = model['scaler']
    # Initialize color_list
	get_color_list.color_list = []

#4 TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
