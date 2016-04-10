#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tooltip_listener");
  ros::NodeHandle nh_("~");

  std::string tooltip_frame;
  std::string base_frame;
  std::string topic_out;
  
  if(!nh_.getParam("tooltip_frame", tooltip_frame))
    tooltip_frame = "/lbr4_7_link";
  
  if(!nh_.getParam("base_frame", base_frame))
    base_frame = "/base_link";

  if(!nh_.getParam("topic_out", topic_out))
    topic_out = "/kuka/tooltip_position";

  ros::Publisher tooltip_position_pub = nh_.advertise<geometry_msgs::PointStamped>(topic_out, 10);

  tf::TransformListener listener;

  ros::Rate rate(100.0);

  while (nh_.ok()){
    tf::StampedTransform transform;
    geometry_msgs::PointStamped tooltip_pos;
    try {
        listener.waitForTransform(base_frame,tooltip_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(base_frame,tooltip_frame, ros::Time(0), transform);
        tooltip_pos.header.stamp=ros::Time::now();
        tooltip_pos.header.frame_id=base_frame;
        tooltip_pos.point.x = transform.getOrigin().x();
        tooltip_pos.point.y = transform.getOrigin().y();
        tooltip_pos.point.z = transform.getOrigin().z();
        tooltip_position_pub.publish(tooltip_pos);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    rate.sleep();
  }
  return 0;
}
