#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class ScanToCloudNode{

    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string frame_;
    std::string scan_topic_;
    std::string cloud_topic_;

public:
    ScanToCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh): tf_buffer_(), tf_listener_(tf_buffer_){
        pnh.param<std::string>("frame", frame_, "laser");
        
        pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh.param<std::string>("cloud_topic", cloud_topic_, "/cloud");

        scan_sub_ = nh.subscribe(scan_topic_, 10, &ScanToCloudNode::scanCallback, this);
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 10);

        ROS_INFO("scan_to_cloud node started. Listening to %s, publishing %s in frame %s",scan_topic_.c_str(), cloud_topic_.c_str(), frame_.c_str());
    }

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
        sensor_msgs::PointCloud2 cloud;

        try{
            projector_.projectLaser(*scan_msg, cloud);

            sensor_msgs::PointCloud2 cloud_out;
            tf_buffer_.transform(cloud, cloud_out, frame_, ros::Duration(0.1));
            cloud_pub_.publish(cloud_out);
        }catch (tf2::TransformException& ex){
            ROS_WARN_STREAM_THROTTLE(1.0, "Transform failed: " << ex.what());
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "scan_to_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ScanToCloudNode node(nh, pnh);

    ros::spin();
    return 0;
}
