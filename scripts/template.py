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
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    #print pcl_msg
    # TODO: Convert ROS msg to PCL data
    ros_cloud_objects = ros_to_pcl(pcl_msg)
    ros_cloud_table = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling

    # Create a VoxelGrid filter object for our input point cloud
    vox_objects = ros_cloud_objects.make_voxel_grid_filter()
    vox_table = ros_cloud_table.make_voxel_grid_filter()


    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size  
    vox_objects.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    vox_table.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    
    vox_objects = vox_objects.filter()
    vox_table = vox_table.filter()

    # TODO: PassThrough Filter
    
    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough_objects = vox_objects.make_passthrough_filter()
    passthrough_table = vox_table.make_passthrough_filter()
    
    filter_axis = 'z'
    passthrough_objects.set_filter_field_name(filter_axis)
    passthrough_table.set_filter_field_name(filter_axis)
    
    axis_min_obj = 0.78
    axis_max_obj = 1.2

    axis_min_table = 0.1
    axis_max_table = 0.78
    passthrough_objects.set_filter_limits(axis_min_obj, axis_max_obj)
    passthrough_table.set_filter_limits(axis_min_table, axis_max_table)
    
    passthrough_objects = passthrough_objects.filter()
    passthrough_table = passthrough_table.filter()


    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
    seg_objects = passthrough_objects.make_segmenter()
    seg_table = passthrough_table.make_segmenter()
    
    # Set the model you wish to fit
    seg_objects.set_model_type(pcl.SACMODEL_PLANE)
    seg_objects.set_method_type(pcl.SAC_RANSAC)
    seg_table.set_model_type(pcl.SACMODEL_PLANE)
    seg_table.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 1
    seg_objects.set_distance_threshold(max_distance)
    seg_table.set_distance_threshold(max_distance)


    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers_objects, coefficients_objects = seg_objects.segment()
    inliers_table, coefficients_table = seg_table.segment()


    # TODO: Extract inliers and outliers

    extracted_inliers_objects = passthrough_objects.extract(inliers_objects, negative=False)
    extracted_inliers_table = passthrough_table.extract(inliers_table, negative=False)

    # TODO: Euclidean Clustering

    white_cloud = XYZRGB_to_XYZ(extracted_inliers_objects) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()


    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.025)
    ec.set_MinClusterSize(13)
    ec.set_MaxClusterSize(1500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    
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
    ros_cloud_objects = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(extracted_inliers_table)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        pcl_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        # Extract histogram features
        chists = compute_color_histograms(sample_cloud, using_hsv=False)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

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


# TODO: ROS node initialization
rospy.init_node('clustering', anonymous=True)
# TODO: Create Subscribers
pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
# TODO: Create Publishers
pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
# Initialize color_list
get_color_list.color_list = []

# TODO: Spin while node is not shutdown
while not rospy.is_shutdown():
    rospy.spin()
   
