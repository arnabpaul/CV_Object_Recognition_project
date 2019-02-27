#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

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

    # TODO: Voxel Grid Downsampling
	# Create a VoxelGrid filter object for our input point cloud
    vox = ros_to_pcl(pcl_msg).make_voxel_grid_filter()

	# Choose a voxel (also known as leaf) size
	# Note: this (1) is a poor choice of leaf size
	# Experiment and find the appropriate size!
    LEAF_SIZE = 0.01 #reasonable voxel(leaf) size

	# Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

	# Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    # TODO: PassThrough Filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

	# Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.77
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

	# Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()


    # TODO: RANSAC Plane Segmentation
	# Create the segmentation object
    seg = cloud_filtered.make_segmenter()

	# Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

	# Max distance for a point to be considered fitting the model
	# Experiment with different values for max_distance
	# for segmenting the table
    max_distance = 0.01 #reasonable choice
    seg.set_distance_threshold(max_distance)

	# Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    def XYZRGB_to_XYZ(XYZRGB_cloud):
    	""" Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes color $

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
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(0.1)
    ec.set_MaxClusterSize(1200)
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
    #def XYZ_to_XYZRGB(XYZ_cloud, color):
    	""" Converts a PCL XYZ point cloud to a PCL XYZRGB point cloud

        	All returned points in the XYZRGB cloud will be the color indicated
        	by the color parameter.

        	Args:
            	XYZ_cloud (PointCloud_XYZ): A PCL XYZ point cloud
            	color (list): 3-element list of integers [0-255,0-255,0-255]

        	Returns:
            	PointCloud_PointXYZRGB: A PCL XYZRGB point cloud
    	"""
    	#XYZRGB_cloud = pcl.PointCloud_PointXYZRGB()
    	#points_list = []

    	#float_rgb = rgb_to_float(color)

    	#for data in XYZ_cloud:
         #   points_list.append([data[0], data[1], data[2], float_rgb])

    	#XYZRGB_cloud.from_list(points_list)
    	#return XYZRGB_cloud
    #cluster_cloud = XYZ_to_XYZRGB(ec,color_cluster_point_list)
    cluster_cloud=pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
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

    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
     rospy.spin()
