#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class ScanToCloudNode{

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    ros::Timer poll_timer_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string frame_;
    std::string scan_topic_;
    std::string cloud_topic_;
    double poll_period_;
    bool subscribed_ = false;

public:
    ScanToCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh): nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_){
        pnh.param<std::string>("frame", frame_, "laser");
        
        pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh.param<std::string>("cloud_topic", cloud_topic_, "/cloud");
        pnh.param<double>("poll_period", poll_period_, 1.0);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 10);

        poll_timer_ = nh_.createTimer(ros::Duration(poll_period_), &ScanToCloudNode::pollCallback, this);

        ROS_INFO("scan_to_cloud node started. Will project %s to %s in frame %s while %s has subscribers",scan_topic_.c_str(), cloud_topic_.c_str(), frame_.c_str(), cloud_topic_.c_str());
    }

private:
    void pollCallback(const ros::TimerEvent&){
        bool wanted = cloud_pub_.getNumSubscribers() > 0;

        if (wanted && !subscribed_){
            scan_sub_ = nh_.subscribe(scan_topic_, 10, &ScanToCloudNode::scanCallback, this);
            subscribed_ = true;
            ROS_INFO("scan_to_cloud: %s has subscribers, listening to %s", cloud_topic_.c_str(), scan_topic_.c_str());
        }else if (!wanted && subscribed_){
            scan_sub_.shutdown();
            subscribed_ = false;
            ROS_INFO("scan_to_cloud: going into standby mode");
        }
    }

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
