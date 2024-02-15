#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Define a global variable to hold the most recent point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert ROS PointCloud2 message to PCL point cloud
    pcl::fromROSMsg(*msg, *g_pcl_cloud);
}

void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes_msg)
{
    // Iterate through the list of bounding boxes
    for (const auto& bounding_box : bounding_boxes_msg->bounding_boxes)
    {
        // Check if the object is a chair
        if (bounding_box.Class == "chair")
        {
            ROS_INFO("Prob: %f", bounding_box.probability);
            ROS_INFO("x_min %ld", bounding_box.xmin);
            ROS_INFO("x_max %ld", bounding_box.xmax);
            ROS_INFO("y_min %ld", bounding_box.ymin);
            ROS_INFO("y_max %ld", bounding_box.ymax);
            // Extract points within the bounding box
            std::vector<float> depths;
            
            float maximum_x = 0;
            for (size_t i = 0; i < g_pcl_cloud->size(); ++i)
            {
            	// ROS_INFO("x: %f", g_pcl_cloud->points[i].x);
            	if (g_pcl_cloud->points[i].x > maximum_x) {
            	    maximum_x = g_pcl_cloud->points[i].x;
            	}
            	
            	ROS_INFO("x: %f", g_pcl_cloud->points[i].x);
                if (g_pcl_cloud->points[i].x >= bounding_box.xmin && g_pcl_cloud->points[i].x <= bounding_box.xmax &&
                    g_pcl_cloud->points[i].y >= bounding_box.ymin && g_pcl_cloud->points[i].y <= bounding_box.ymax)
                {
                    ROS_INFO("In box");
                    depths.push_back(g_pcl_cloud->points[i].z);
                }
            }
            //ROS_INFO("Maximum_x : %f", maximum_x );

            // Calculate the average depth of points within the bounding box
            double sum_depth = 0.0;
            for (const auto& depth : depths)
            {
                sum_depth += depth;
            }
            double average_depth = depths.empty() ? 0.0 : sum_depth / depths.size();

            // Process the average depth as needed
            ROS_INFO("Average depth of points within the bounding box: %f", sum_depth);

        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "mattbot_synthesize");
    ros::NodeHandle nh;

    // Create a subscriber for the point cloud topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, pointCloudCallback);
    
    // Subscribe to the bounding boxes topic
    ros::Subscriber bounding_boxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, boundingBoxesCallback);


    // Spin to keep the node alive and continue receiving messages
    ros::spin();

    return 0;
}

