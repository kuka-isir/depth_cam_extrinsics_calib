
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
// PCL-ROS auto type conversions
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <dynamic_reconfigure/server.h>
#include <depth_cam_extrinsics_calib/calibConfig.h>

depth_cam_extrinsics_calib::calibConfig config; 
ros::Publisher point_st_pub,cloud_pub;

// From https://github.com/joshvillbrandt/mecanumbot-ros-pkg/blob/master/src/ball_tracker.cpp
namespace cloud_helpers
{
    pcl::PointXYZHSV
    getCloudAverage(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {

        pcl::PointXYZHSV avg;
        avg.x = 0; avg.y = 0; avg.z = 0; avg.h = 0; avg.s = 0; avg.v = 0;

        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            if(!isnan(cloud->points[i].x) && !isnan(cloud->points[i].y) && !isnan(cloud->points[i].z)) {
                avg.x += cloud->points[i].x;
                avg.y += cloud->points[i].y;
                avg.z += cloud->points[i].z;
                avg.h += cloud->points[i].h;
                avg.s += cloud->points[i].s;
                avg.v += cloud->points[i].v;
            }
        }

        avg.x /= cloud->points.size();
        avg.y /= cloud->points.size();
        avg.z /= cloud->points.size();
        avg.h /= cloud->points.size();
        avg.s /= cloud->points.size();
        avg.v /= cloud->points.size();

        return avg;
    }
    
  void PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
    {
        out.width = in.width;
        out.height = in.height;
        out.header = in.header;
        for (size_t i = 0; i < in.points.size (); i++) {
            pcl::PointXYZRGB p;
            pcl::PointXYZHSVtoXYZRGB (in.points[i], p);
            p.x = in.points[i].x;
            p.y = in.points[i].y;
            p.z = in.points[i].z;
            out.points.push_back (p);
        }
    }
    void PointCloudXYZRGBtoXYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out)
    {
        out.width = in.width;
        out.height = in.height;
        out.header = in.header;
        for (size_t i = 0; i < in.points.size (); i++) {
            pcl::PointXYZHSV p;
            pcl::PointXYZRGBtoXYZHSV (in.points[i], p);
            p.x = in.points[i].x;
            p.y = in.points[i].y;
            p.z = in.points[i].z;
            out.points.push_back (p);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr>
    extractClusters(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud, std::vector<pcl::PointIndices::Ptr> cluster_indices)
    {
        // set up extractor
        pcl::ExtractIndices<pcl::PointXYZHSV> extract;
        extract.setInputCloud(cloud);
        extract.setNegative(false);

        // set up cloud vector
        std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> clusters;
        clusters.resize(cluster_indices.size());

        // extract clusters
        for(size_t i = 0; i < cluster_indices.size(); i++) {
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZHSV>);
            extract.setIndices(cluster_indices[i]);
            extract.filter(*cluster);
            clusters[i] = cluster;
        }
        return clusters;
    }
}
void callback(const pcl::PCLPointCloud2ConstPtr& cloud_in){
	// convert to RGB from input
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*cloud_in, *cloud_in2);

	// convert to HSV for processing
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
	cloud_helpers::PointCloudXYZRGBtoXYZHSV(*cloud_in2, *cloud_filtered);

	// filter saturation
	pcl::PassThrough<pcl::PointXYZHSV> pass;
	
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("h");
	pass.setFilterLimits(config.h_min, config.h_max);
	pass.setFilterLimitsNegative (false);
	pass.filter (*cloud_filtered);
	
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("s");
	pass.setFilterLimits(config.s_min, config.s_max);
	pass.setFilterLimitsNegative (false);
	pass.filter (*cloud_filtered);

	// filter value
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits(config.v_min, config.v_max);
	pass.setFilterLimitsNegative (false);
	pass.filter (*cloud_filtered);

	if(config.use_StatisticalOutlierRemoval)
	{
	  // statistical outlier filter (Useful for carpet)
	  pcl::StatisticalOutlierRemoval<pcl::PointXYZHSV> sor;
	  sor.setInputCloud(cloud_filtered);
	  sor.setMeanK(config.mean_k);
	  sor.setStddevMulThresh(0.01); // smaller is more restrictive
	  sor.filter(*cloud_filtered);
	}
	
	if(cloud_filtered->width*cloud_filtered->height > 0) {
	  pcl::PointXYZHSV avg = cloud_helpers::getCloudAverage(cloud_filtered);
	  geometry_msgs::PointStamped pt_out;
	  pt_out.header = pcl_conversions::fromPCL(cloud_in->header);
	  pt_out.point.x = avg.x;
	  pt_out.point.y = avg.y;
	  pt_out.point.z = avg.z;
	  point_st_pub.publish(pt_out);

	}

	if(cloud_pub.getNumSubscribers() > 0) {
	    cloud_filtered->header = cloud_in->header;
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
	    cloud_helpers::PointCloudXYZHSVtoXYZRGB(*cloud_filtered, *cloud_out);
	    cloud_pub.publish(*cloud_out);
	}
}

void callbackConfig(depth_cam_extrinsics_calib::calibConfig& _config, uint32_t level)
{
  ROS_INFO("Updating config : H[%d,%d] S[%f,%f] V[%f,%f]",_config.h_min,_config.h_max,_config.v_min,_config.v_max,_config.s_min,_config.s_max);
  config = _config;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pcl_hsv_seg");
		
	dynamic_reconfigure::Server<depth_cam_extrinsics_calib::calibConfig> srv;
	dynamic_reconfigure::Server<depth_cam_extrinsics_calib::calibConfig>::CallbackType f;
	f = boost::bind(&callbackConfig, _1, _2);
	srv.setCallback(f);

	ros::NodeHandle nh,nh_priv("~");
	std::string topic_in("/camera/depth_registered/points");
	
	if(!nh_priv.getParam("topic_in",topic_in))
	{
	  ROS_WARN("Using default topic : %s",topic_in.c_str());
	}
	
	ros::Subscriber cloud_sub = nh.subscribe(topic_in, 1, callback);
	cloud_pub = nh_priv.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("hsv_filtered",1);
	point_st_pub = nh_priv.advertise<geometry_msgs::PointStamped>("average_pt",10);
	ros::spin();
	return 0;
}