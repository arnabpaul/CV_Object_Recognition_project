# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('pr2_view.pcd')


# Voxel Grid filter
# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

# Choose a voxel (also known as leaf) size
# Note: this (1) is a poor choice of leaf size
# Experiment and find the appropriate size!
LEAF_SIZE = 0.003 #reasonable voxel(leaf) size

# Set the voxel (or leaf) size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
#filename = 'voxel_downsampled.pcd'
#pcl.save(cloud_filtered, filename)

# PassThrough filter
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
#filename = 'pass_through_filtered1.pcd'
#pcl.save(cloud_filtered, filename)

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
#filename = 'statistical_outlier.pcd'
#pcl.save(cloud_filtered, filename)

# RANSAC plane segmentation
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

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Save pcd for table
#filename= 'Table.pcd'
#pcl.save(extracted_inliers, filename)


# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_outliers.pcd'
pcl.save(extracted_outliers, filename)

# Save pcd for tabletop objects
#filename= 'Objects.pcd'
#pcl.save(extracted_outliers, filename)
def XYZRGB_to_XYZ(XYZRGB_cloud):
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
cluster_cloud=pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
filename = 'cluster_cloud_pr.pcd'
pcl.save(cluster_cloud, filename)

if __name__ == '__main__':
# Initialize color_list
    get_color_list.color_list = []
