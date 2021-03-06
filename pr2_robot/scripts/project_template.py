#!/usr/bin/env python

# Import modules
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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

def Statistical_Outlier_Filtering(cloud):
    	fil = cloud.make_statistical_outlier_filter()
    	fil.set_mean_k(15)
    	fil.set_std_dev_mul_thresh(0.03)
   	cloud_fliter=fil.filter()
	filename = 'Statistical_Outlier_Filtering.pcd'
	pcl.save(cloud_fliter, filename)
	return cloud_fliter
        
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
	ec.set_ClusterTolerance(0.05)    #0.05 for all
	ec.set_MinClusterSize(60)        #130 for test1 , 60 for test3 and 118 for test 2  
	ec.set_MaxClusterSize(3000)   #3000 for all
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
    # TODO: Statistical Outlier Filtering
	cloud_fliter=Statistical_Outlier_Filtering(cloud)
# TODO: Voxel Grid Downsampling
	#create a Voxel filter  object forourinput point cloud
	cloud_filtered_v=voxel_filter(cloud_fliter)
# TODO: PassThrough Filter
	#Create a PassThrough filter object
	#cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'x',0.08,0.6) #for test2 (0.08,0.6)
	cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'y',-0.4,0.4)#for all
	cloud_filtered_pass=passthrough_filter(cloud_filtered_v,'z',0.6,0.91) #for test2 (0.6,0.8)
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
            	chists = compute_color_histograms(ros_cluster, using_hsv=True,nbins=32)#False
            	normals = get_normals(ros_cluster)
            	nhists = compute_normal_histograms(normals,nbins=16)
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

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    	detected_objects_pub.publish(detected_objects)
    # Publish the list of detected objects
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    	try:
       		pr2_mover(detected_objects)
   	except rospy.ROSInterruptException:
        	pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
	output_list = []
	centroid_list = []
	drop_param={}

    # TODO: Get/Read parameters
	object_list_param = rospy.get_param('/object_list')
	
    	dropbox_list_param = rospy.get_param('/dropbox')
	ros_pick_pose = Pose()
        ros_pick_pose.position.x = 0
        ros_pick_pose.position.y = 0
        ros_pick_pose.position.z = 0

        #set orientation to 0
        ros_pick_pose.orientation.x = 0
        ros_pick_pose.orientation.y = 0
        ros_pick_pose.orientation.z = 0
        ros_pick_pose.orientation.w = 0

        #set place pose orientation to 0
        ros_place_pose = Pose()
        ros_place_pose.orientation.x = 0
        ros_place_pose.orientation.y = 0
        ros_place_pose.orientation.z = 0
        ros_place_pose.orientation.w = 0
	

    # TODO: Parse parameters into individual variables
	for pos in dropbox_list_param:
		drop_param[pos['name']] = pos['position']
	for object_params in object_list_param:
        	object_name = object_params['name']
		#print ("object_name",object_name)
        	object_group = object_params['group']
		for i, object_val in enumerate(object_list):
		    if object_name == object_val.label:
			labels = []
			centroids = [] # to be list of tuples (x, y, z)
    			labels.append(object_val.label)
			points_arr = ros_to_pcl(object_val.cloud).to_array()
			centroid_list=np.mean(points_arr, axis=0)[:3]
			centroid = [np.asscalar(x) for x in centroid_list]
		
        		# TODO: Create 'place_pose' for the object
		        ros_pick_pose = Pose()
       		        ros_pick_pose.position.x = centroid[0]
	                ros_pick_pose.position.y = centroid[1]
	                ros_pick_pose.position.z = centroid[2]
			#print ("object_name :",labels)		      	
		        break
		ros_scene_num = Int32()
		# TODO: Figure out what parameter holds the scene data
		test_num =3
            	ros_scene_num.data = test_num

            	# Assign the object name
            	ros_object_name = String()
           	ros_object_name.data = object_name
           	 # Assign the arm that'll be used to pickup the object
           	ros_arm_to_use = String()
            	if object_group == 'green':
                	# The green bin is on the robot's right
                	ros_arm_to_use.data = 'right'
            	else:
                	# The red bin is on the robot's left
                	ros_arm_to_use.data = 'left'  
            	# TODO: Add a random offset to the dropbox's position

            	# Assign the dropbox pose
            	ros_place_pos = Pose()
            	ros_place_pos.position.x = drop_param[ros_arm_to_use.data][0]
            	ros_place_pos.position.y = drop_param[ros_arm_to_use.data][1]
            	ros_place_pos.position.z = drop_param[ros_arm_to_use.data][2]
		# Add the object's yaml dict to the output_list
            	obj_yaml_dict = make_yaml_dict(ros_scene_num, ros_arm_to_use,
                                           ros_object_name, ros_pick_pose,
                                           ros_place_pos)
		output_list.append(obj_yaml_dict)
		print('processed ', ros_object_name.data)

	
		

    # TODO: Rotate PR2 in place to capture side tables for the collision map

#    # TODO: Loop through the pick list
#	
#        # Wait for 'pick_place_routine' service to come up
       		rospy.wait_for_service('pick_place_routine')

#       try:
 #          pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

#            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

 #          print ("Response: ",resp.success)

  #     except rospy.ServiceException, e:
   #        print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
	yaml_filename = "output_" + str(ros_scene_num.data) + ".yaml"

    	send_to_yaml(yaml_filename, output_list)

if __name__ == '__main__':
#1 TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)
#2 TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
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
