#ifndef CLUSTER_EXTRACTION_H
#define CLUSTER_EXTRACTION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/geometry.h>

#include <std_msgs/ColorRGBA.h>
#include <point_cloud_clustering/clusterInfo.h>

#include <pcl_ros/filters/passthrough.h>

class Cluster
{
    public:
    
    int id = -1;
    std_msgs::ColorRGBA color;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointXYZ centroid;
    pcl::PointXYZ farthest_point;
    double radius;
    double cyl_radius;

};

class ClusterExtractionParams
{
    public:
    double downsample_leaf_size;
    int ransac_max_iterations;
    double ransac_distance_threshold;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    double same_obj_thres;
    int same_obj_size;
    double passthrough_min;
    double passthrough_max;
    int avg;

};

class ClusterExtraction
{
    public:
    // Constructor/Destructor
    ClusterExtraction();
    ~ClusterExtraction();

    // Functions
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void downsampleCloud();
    void removePlaner();
    void findClusters();
    void passthrough();
    std::vector<Cluster> getClusters();
    void calcClusterInfo();
    point_cloud_clustering::clusterInfo getClusterInfo();
    void matchClusters();
    void avgcluster();
    

    // Data
    ClusterExtractionParams params;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cloud_filtered,cloud_pass;
     pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f, cloud_plane;

    int avg = 0;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

    std::vector<Cluster> clusters;
    std::vector<Cluster> all_clusters;

};

#endif