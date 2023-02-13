#include <ros/ros.h>
#include <point_cloud_clustering/cluster_extraction.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <signal.h>
#include <stdlib.h>
#include <point_cloud_clustering/clusterInfo.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/console/time.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
using namespace std;

//adding lib for saving point cloud///////////////////////////////////////////////////////////////////
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <thread>
//////////////////////////////////////////////////////////////////////////////////////////////////////

tf2_ros::Buffer tfBuffer;
sensor_msgs::PointCloud2 pcd_source_world;
class PointCloudClustering {
    private:

    ros::Publisher visPub;
    ros::Publisher voxPub;
    ros::Publisher passPub;
    ros::Publisher clusterInfoPub;
    ros::Publisher cylVisPub;
    ros::Subscriber pointCloudSub;
    ClusterExtraction ce;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud;
    pcl::PCLPointCloud2 pcl_pc2;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    

    public:

    

    bool to_do = true;

         
       
    PointCloudClustering(ros::NodeHandle *nh) {
        // pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
        
        visPub = nh->advertise<sensor_msgs::PointCloud2>("/clusters", 1);
        voxPub = nh->advertise<sensor_msgs::PointCloud2>("/voxels", 1);

        passPub = nh->advertise<sensor_msgs::PointCloud2>("/pass", 1);

        
        clusterInfoPub = nh->advertise<point_cloud_clustering::clusterInfo>("/cluster_info", 1);
        
        pointCloudSub = nh->subscribe("/octomap_point_cloud_centers", 1, &PointCloudClustering::pointCloudcb, this); 

        cylVisPub = nh->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        nh->param<double>("/point_cloud_clustering_node/downsample_leaf_size", ce.params.downsample_leaf_size, 0.01);
        nh->param<int>("/point_cloud_clustering_node/ransac_max_iterations", ce.params.ransac_max_iterations, 100);
        nh->param<double>("/point_cloud_clustering_node/ransac_distance_threshold", ce.params.ransac_distance_threshold, 0.02);
        nh->param<double>("/point_cloud_clustering_node/cluster_tolerance", ce.params.cluster_tolerance, 0.02);
        nh->param<int>("/point_cloud_clustering_node/min_cluster_size", ce.params.min_cluster_size, 100);
        nh->param<int>("/point_cloud_clustering_node/max_cluster_size", ce.params.max_cluster_size, 25000);
        nh->param<int>("/point_cloud_clustering_node/avg", ce.params.avg, 2);
        nh->param<double>("/point_cloud_clustering_node/same_obj_thres", ce.params.same_obj_thres, 0.5);
        nh->param<int>("/point_cloud_clustering_node/same_obj_size", ce.params.same_obj_size, 50);

        nh->param<double>("/point_cloud_clustering_node/passthrough_min", ce.params.passthrough_min, 0.0);
        nh->param<double>("/point_cloud_clustering_node/passthrough_max", ce.params.passthrough_max, 7.0);




    }

    void pointCloudcb(const sensor_msgs::PointCloud2ConstPtr pcd_source_ptr) {

        pcl::console::TicToc time;
        time.tic();

        pcl::PCLPointCloud2 pcl_pcd2;
        // pcd_source_world = *pcd_source_ptr;
        // pcd_source_world.header.frame_id = "optimized_pc";
        geometry_msgs::TransformStamped transform_source, transform_target;
          
        
        try {
            transform_source = tfBuffer.lookupTransform("optimized_body",
                                                        "optimized_pc",
                                                        ros::Time(0));
        
            tf2::doTransform(*pcd_source_ptr, pcd_source_world, transform_source);

            passPub.publish(pcd_source_world);
        
            // pcl_conversions::toPCL(pcd_source_world, pcl_pcd2);
            // cur_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::fromPCLPointCloud2(pcl_pcd2,*cur_cloud);
            
            }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            
            return;
            }

        // processCloud();
        ROS_WARN("done in %lf seconds",time.toc() / 1000);

    }

    void processCloud() {

        

        
        sensor_msgs::PointCloud2 avg_msg,pass_msg;

        if (!cur_cloud)
            return;
        ce.setInputCloud(cur_cloud);

        ce.passthrough();

     
        
       

        pcl::toROSMsg(*ce.cloud_filtered, pass_msg);
        
        
        ce.downsampleCloud();

        ce.cloud_a = ce.cloud_a + *ce.cloud_filtered;

        *ce.cloud_filtered = ce.cloud_a;

        ce.downsampleCloud();
        ce.cloud_a = *ce.cloud_filtered;
        


        //ce.avgcluster();

        pcl::toROSMsg(ce.cloud_a, avg_msg);

        avg_msg.header.frame_id = "world";
        pass_msg.header.frame_id = "world";

        voxPub.publish(avg_msg);
        passPub.publish(pass_msg);
        //ce.removePlaner();
        //ce.findClusters();

        //pubVisualisation();
        //pubClusterInfo();
         //pubCylinder();
        
    }

    void pubClusterInfo() {
        clusterInfoPub.publish(ce.getClusterInfo());
    }

    void pubCylinder() {
        visualization_msgs::MarkerArray markers;
        for (auto& cluster : ce.getClusters()) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_link";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = cluster.id;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cluster.centroid.x;
            marker.pose.position.y = cluster.centroid.y;
            marker.pose.position.z = cluster.centroid.y;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = cluster.cyl_radius;
            marker.scale.y = cluster.cyl_radius;
            marker.scale.z = 100;
            marker.color.a = 0.8; // Don't forget to set the alpha!
            marker.color.r = cluster.color.r;
            marker.color.g = cluster.color.g;
            marker.color.b = cluster.color.b;
            markers.markers.push_back(marker);
        }
        cylVisPub.publish(markers);
    }

    void pubVisualisation() {

        sensor_msgs::PointCloud2 vis_msg;

        vis_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (auto& cluster : ce.getClusters())
        {
            pcl::PointXYZRGB p;
            // std_msgs::ColorRGBA color;
            // color.r = rand() % 154 + 100;
            // color.g = rand() % 154 + 100;
            // color.b = rand() % 154 + 100;
            for (size_t i = 0; i < cluster.cloud->size(); i++) {
                // cout << "H" << endl;
                p.x = cluster.cloud->points[i].x;
                p.y = cluster.cloud->points[i].y;
                p.z = cluster.cloud->points[i].z;
                p.r = cluster.color.r;
                p.g = cluster.color.g;
                p.b = cluster.color.b;
                vis_cloud->push_back(p);
            }
        }

        // pcl::toROSMsg(*(ce.cloud_filtered), vis_msg);


        // ROS_WARN("Cluster Count: %ld", ce.getClusters().size());
        // ROS_WARN("Vis Cloud size: %ld", vis_cloud->size());
        pcl::toROSMsg(*vis_cloud, vis_msg);
        vis_msg.header.frame_id = "camera_link";
        // ROS_WARN("%s", vis_msg.header.frame_id.c_str());
        visPub.publish(vis_msg);

    }
};

void command()
{
    while(1)
    {
         char c = getchar();
        if (c == 's')
        {

            pcl::PointCloud<pcl::PointXYZ> cur_cloud;
            pcl::PCLPointCloud2 pcl_pcd2;


            pcl_conversions::toPCL(pcd_source_world, pcl_pcd2);
            pcl::fromPCLPointCloud2(pcl_pcd2,cur_cloud);


            pcl::io::savePCDFileASCII ("test_pcd.pcd", cur_cloud);


             

            std::cerr << "Saved " << cur_cloud.size () << " data points to test_pcd.pcd." << std::endl;

           }
    }

  }

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    




    tf2_ros::TransformListener tfListener(tfBuffer);
    PointCloudClustering nc = PointCloudClustering(&nh);
    signal(SIGINT, mySigintHandler);
    
//////////keyboard input to save point cloud ////////////////////////////////////
     std::thread keyboard_command_process;
     keyboard_command_process = std::thread(command);


    ros::spin();
}

// int main (int argc, char **argv)
// {
//     ros::init(argc, argv, "cluster_extraction_node");
//     ros::NodeHandle nh;

//     ClusterExtraction ce;
//     // Read in the cloud data
//     pcl::PCDReader reader;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//     reader.read ("table_scene_lms400.pcd", *cloud);
//     std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

//     ce.setInputCloud(cloud);
//     ce.downsampleCloud();
//     std::cout << "PointCloud after filtering has: " << ce.cloud_filtered->size () << " data points." << std::endl; //*
//     ce.removePlaner();
//     ce.findClusters();

//     pcl::PCDWriter writer;
//     int j = 0;

//     for (auto &cluster : ce.getClusters())
//     {
//         std::cout << "PointCloud representing the Cluster: " << cluster.cloud->size () << " data points." << std::endl;
//         std::stringstream ss;
//         ss << "cloud_cluster_" << j << ".pcd";
//         writer.write<pcl::PointXYZ> (ss.str (), *(cluster.cloud), false); //*
//         j++;

//         std::cout << "PointCloud Centroid: " << cluster.centroid << std::endl;

//         std::cout << "Radius: " << cluster.radius << std::endl;
//         std::cout << "Farthest Point: " << cluster.farthest_point << std::endl;
//     }

//     return (0);
// }