#include <point_cloud_clustering/cluster_extraction.h>
#include <pcl_ros/filters/passthrough.h>

#include <iostream>
#include <stdlib.h>

using namespace std;

ClusterExtraction::ClusterExtraction()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_f.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_plane.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

ClusterExtraction::~ClusterExtraction()
{

}

void ClusterExtraction::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	this->cloud_filtered = cloud;
}

void ClusterExtraction::downsampleCloud()
{
	if (!this->cloud_filtered)
	{
		std::cout << "No Input Cloud!" << std::endl;
		return;
	}

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud_filtered);
	double leafSize = params.downsample_leaf_size;
	vg.setLeafSize (leafSize, leafSize, leafSize);
	vg.filter (*cloud_filtered);
}
void ClusterExtraction::passthrough()
{
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(params.passthrough_min,params.passthrough_max);
	pass.filter(*cloud_filtered);
}
void ClusterExtraction::avgcluster(){
	
	//cloud_a += cloud_b;
	if(avg >params.avg)
	{
		avg = 0;
		cloud_a = cloud_c;
	}
	else {
		cloud_a += *cloud_filtered;
		*cloud_filtered = cloud_a;
	}
	avg++;
	  
}
void ClusterExtraction::removePlaner()
{
	if (!cloud_filtered)
	{
		std::cout << "No Filtered Cloud!" << std::endl;
		return;
	}

	// Create the segmentation object for the planar model and set all the parameters
	inliers.reset(new pcl::PointIndices);
	coefficients.reset(new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (params.ransac_max_iterations);
	seg.setDistanceThreshold (params.ransac_distance_threshold);


	int nr_points = (int) cloud_filtered->size ();

	while (cloud_filtered->size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;

	}
}

void ClusterExtraction::findClusters()
{
	if (!this->cloud_filtered)
	{
		std::cout << "No Filtered Cloud!" << std::endl;
		return;
	}

	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	cluster_indices.clear();

	ec.setClusterTolerance (params.cluster_tolerance); // 2cm
	ec.setMinClusterSize (params.min_cluster_size);
	ec.setMaxClusterSize (params.max_cluster_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	ROS_WARN( "cluster_indices.size() =%d ",cluster_indices.size());

	clusters.clear();
	clusters.resize(cluster_indices.size());
	for (size_t i = 0; i < cluster_indices.size (); ++i)
	{
		clusters[i].cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud (*cloud_filtered, cluster_indices[i], *(clusters[i].cloud));
		clusters[i].cloud->width = clusters[i].cloud->size ();
		clusters[i].cloud->height = 1;
		clusters[i].cloud->is_dense = true;
	}

	calcClusterInfo();
	 ROS_WARN("hello");
	//matchClusters();
}

void ClusterExtraction::matchClusters()
{
	int taken[all_clusters.size()];
	for (int i = 0; i < all_clusters.size(); ++i)
	{
		taken[i] = -1;
	}
	for (size_t i = 0; i < clusters.size(); ++i)
	{
		for (size_t j = 0; j < all_clusters.size(); ++j)
		{
			if (taken[i] != -1)
				continue;

			double distance = pcl::geometry::distance(all_clusters[j].centroid, clusters[i].centroid);
			if (distance < params.same_obj_thres)
			{
				int size_diff = clusters[i].cloud->size() - all_clusters[j].cloud->size();
				if (size_diff < 0)
					size_diff *= -1;
				// if (size_diff > params.same_obj_size)
				// 	continue;

				clusters[i].id = j;
				clusters[i].color = all_clusters[j].color;

				all_clusters[j].cloud = clusters[i].cloud;
				all_clusters[j].centroid = clusters[i].centroid;
				all_clusters[j].farthest_point = clusters[i].farthest_point;
				all_clusters[j].radius = clusters[i].radius;

				taken[i] = j;

				break;

			}
		}

		if (clusters[i].id == -1)
		{
			clusters[i].id = all_clusters.size();
			clusters[i].color.r = rand() % 154 + 100;
			clusters[i].color.g = rand() % 154 + 100;
			clusters[i].color.b = rand() % 154 + 100;
			all_clusters.push_back(clusters[i]);
		}
	}

}

void ClusterExtraction::calcClusterInfo()
{
	for (size_t i = 0; i < clusters.size(); ++i)
	{
		Eigen::Vector4f centroid_cluster;   
        pcl::compute3DCentroid(*(clusters[i].cloud), centroid_cluster);    
        clusters[i].centroid = pcl::PointXYZ(centroid_cluster[0], centroid_cluster[1], centroid_cluster[2]);
    
		// cout << "ok" << endl;

        // Farthest Point
        double max = -1;
		double max_cyl = -1;
        for (const auto &point : clusters[i].cloud->points)
        {
            double distance = pcl::geometry::distance(point, clusters[i].centroid);
            if (distance > max)
            {
                max = distance;
                clusters[i].farthest_point = point;
            }

			pcl::PointXYZ xy_centroid = clusters[i].centroid;
			xy_centroid.z = 0;

			pcl::PointXYZ xy_point = point;
			xy_point.z = 0;

			double cyl_distance = pcl::geometry::distance(xy_point, xy_centroid);
			if (cyl_distance > max_cyl)
			{
				max_cyl = cyl_distance;
			}
        }
		clusters[i].radius = max;
		clusters[i].cyl_radius = max_cyl;
	}
	// std::cout << "Here" << std::endl;

}

point_cloud_clustering::clusterInfo ClusterExtraction::getClusterInfo()
{
	point_cloud_clustering::clusterInfo cluster_info;
	cluster_info.no_of_clusters = clusters.size();
	for (size_t i = 0; i < clusters.size(); ++i)
	{
		geometry_msgs::Point centroid;
		centroid.x = clusters[i].centroid.x;
		centroid.y = clusters[i].centroid.y;
		centroid.z = clusters[i].centroid.z;
		cluster_info.centroids.push_back(centroid);
		cluster_info.ids.push_back(clusters[i].id);
		cluster_info.radii.push_back(clusters[i].radius);
		cluster_info.cyl_radii.push_back(clusters[i].cyl_radius);
		// cluster_info.colors.push_back(clusters[i].color);
	}
	return cluster_info;
}


std::vector<Cluster> ClusterExtraction::getClusters()
{
	return clusters;
}





