#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import matplotlib.colors
import matplotlib.pyplot as plt
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
    #yaml_dict["pick_pose"] = pick_pose
    #yaml_dict["place_pose"] = place_pose
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)
# TODO: Convert ROS msg to PCL data
def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
              Args:
                 ros_cloud (PointCloud2): ROS PointCloud2 message
              Returns:
                 pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list=[]

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data
def XYZRGB_to_XYZ(XYZRGB_cloud):
    """ Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes col$

            Args:
                XYZRGB_cloud (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

            Returns:
                PointCloud_PointXYZ: A PCL XYZ point cloud
    """
    XYZ_cloud = pcl.PointCloud()
    points_list = []

    for data in XYZRGB_cloud:
        points_list.append([data[0], data[1], data[2]])

    XYZ_cloud.from_list(points_list)
    return XYZ_cloud
# TODO: Convert PCL data to ROS messages
def pcl_to_ros(pcl_array):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message
              Args:
                pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
              Returns:
                PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"
    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                        name="x",
                        offset=0,
                        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                        name="y",
                        offset=4,
                        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                        name="z",
                        offset=8,
                        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                        name="rgb",
                        offset=16,
                        datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))
    ros_msg.data = "".join(buffer)
    return ros_msg
def rgb_to_hsv(rgb_list):
        rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
        hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
        return hsv_normalized
def compute_color_histograms(cloud, using_hsv=True):

            # Compute histograms for the clusters
    point_colors_list = []

            # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

            # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])

            # TODO: Compute histograms
    h_hist = np.histogram(channel_1_vals, bins=32, range=(0, 256))
    s_hist = np.histogram(channel_2_vals, bins=32, range=(0, 256))
    v_hist = np.histogram(channel_3_vals, bins=32, range=(0, 256))

            # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
            # Generate random features for demo mode.  
            # Replace normed_features with your feature vector
            #normed_features = np.random.random(96)
        #normed_features = np.asscalar(hist_features) / np.asscalar(np.sum(hist_features))
    normed_features = hist_features / np.sum(hist_features)
        #normed_features = np.asscalar(normed_features)
    return normed_features
def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []
    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])
        # TODO: Compute histograms of normal values (just like with color)
    norm_x_hist = np.histogram(norm_x_vals, bins=32, range=(0,256))
    norm_y_hist = np.histogram(norm_y_vals, bins=32, range=(0,256))
    norm_z_hist = np.histogram(norm_z_vals, bins=32, range=(0,256))
                # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])).astype(np.float64)
                # Generate random features for demo mode.  
                # Replace normed_features with your feature vector
                #normed_features = np.random.random(96)
    normed_features = hist_features / np.sum(hist_features)
        #normed_features = np.asscalar(hist_features) / np.asscalar(np.sum(hist_features))
        #normed_features = np.asscalar(normed_features)
    return normed_features
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 
    pr2_view = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = pr2_view.make_voxel_grid_filter()

        # Choose a voxel (also known as leaf) size
        # Note: this (1) is a poor choice of leaf size
        # Experiment and find the appropriate size!
    LEAF_SIZE = 0.003 #reasonable voxel(leaf) size

        # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

        # Call the filter function to obtain the resultant downsampled point clo$
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

        # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
	# Finally use the filter function to obtain the resultant point cloud.
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.605
    axis_max = 0.8
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
        # Finally use the filter function to obtain the resultant point cloud.
    ##cloud_filtered = passthrough.filter()
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()

        # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

        # Set threshold scale factor
    x = 1.0

        # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

        # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

        # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

        # Max distance for a point to be considered fitting the model
        # Experiment with different values for max_distance
        # for segmenting the table
    max_distance = 0.015 #reasonable choice
    seg.set_distance_threshold(max_distance)

        # Call the segment function to obtain set of inlier indices and model co$
    inliers, coefficients = seg.segment()
    # TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
        # Set tolerances for distance threshold
        # as well as minimum and maximum cluster size (in points)
        # NOTE: These are poor choices of clustering parameters
        # Your task is to experiment and find values that work for segmenting ob$
    ec.set_ClusterTolerance(0.0090)
    ec.set_MinClusterSize(0.01)
    ec.set_MaxClusterSize(15000)
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

        #cluster_cloud = XYZ_to_XYZRGB(ec,color_cluster_point_list)
    cluster_cloud=pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
	pcl_cluster = extracted_outliers.extract(pts_list)
        # Compute the associated feature vector
	sample_cloud=pcl_to_ros(pcl_cluster)
	chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
	feature = np.concatenate((chists, nhists))
        # Make the prediction
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
        do.cloud = sample_cloud
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
	pr2_mover(detected_objects)
        #pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(objects_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    object_group = String()
    which_arm = String()
    pick_pose = Pose()
    place_pose = Pose()
    labels = []
    centroids = []
    dict_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    drop_list_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables
    for i,objects in enumerate(object_list_param):
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
	if object_group == 'green':
           which_arm = 'right'
        else:
            which_arm = 'left'
    for j,drop_obs in enumerate(drop_list_param):
	drop_pos = drop_list_param[j]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid
	#for object in objects:
    for obj in objects_list:
        labels.append(obj.label)
        points_arr = ros_to_pcl(obj.cloud).to_array()
    	    #centroids.append(np.asscalar(np.mean(points_arr, axis=0)[:3]))
        centroids.append(np.mean(points_arr, axis=0)[:3])
	    #centroids = [type(np.asscalar(i)) for i in centroids]
	    #centroids = type(np.asscalar(centroids))
	X_p = centroids[0]
	#print(X_p)
	c_x=np.asscalar(np.array(X_p[0]))
	c_y=np.asscalar(np.array(X_p[1]))
	c_z=np.asscalar(np.array(X_p[2]))
	#center=[c_x, c_y, c_z]
	X_p[0]=c_x
	X_p[1]=c_y
	X_p[2]=c_z

	#print(centroids)
        # TODO: Create 'place_pose' for the object
	pick_pose.position.x = X_p[0]
	pick_pose.position.y = X_p[1]
	pick_pose.position.z = X_p[2]
	#pick_pose = X_p
	#place_pose = drop_pos
	place_pose.position.x = drop_pos[0]
	place_pose.position.y = drop_pos[1]
	place_pose.position.z = drop_pos[2]
        # TODO: Assign the arm to be used for pick_place
	#if object_group == "green"
         #   which_arm = 'right'
        #else
         #   which_arm = 'left'
	arm_name.data = which_arm
	test_scene_num.data = 1
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
#	def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
#    	    yaml_dict = {}
#    	    yaml_dict["test_scene_num"] = test_scene_num.data
#    	    yaml_dict["arm_name"]  = which_arm.data
#    	    yaml_dict["object_name"] = object_name.data
#    	    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
#    	    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
#    	    return yaml_dict
	#dict_list = []
    for i in range(0, len(object_list_param)):
    		# Populate various ROS messages
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
        # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
        test_scene_num.data = 1
	arm_name.data = which_arm
	#for i,objects in enumerate(object_list_param):
         #   object_name.data = object_list_param[i]['name']
          #  object_group = object_list_param[i]['group']
           # if object_group == 'green':
            #   which_arm = 'right'
           # else:
            #   which_arm = 'left'
	object_name.data[0] = object_list_param[0]['name']
	pick_pose.position.x = X_p[0]
        pick_pose.position.y = X_p[1]
        pick_pose.position.z = X_p[2]
        
        place_pose.position.x = drop_pos[0]
        place_pose.position.y = drop_pos[1]
        place_pose.position.z = drop_pos[2]
            #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
        resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_1.yaml', dict_list)
    #	data_dict = {"object_list": dict_list}
    #	with open('output_1.yaml', 'w') as outfile:
     #       yaml.dump(data_dict, outfile, default_flow_style=False)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # TODO: Load Model From disk
    model = pickle.load(open('model1.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    #Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
     rospy.spin()
