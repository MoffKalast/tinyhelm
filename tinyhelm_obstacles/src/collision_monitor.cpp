#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Twist.h>
#include <tinyhelm_core/MonitorStatus.h>

// Dumb and fast by design: watches raw obstacle detections (no decay, no hit counting) and
// screams EMERGENCY if anything enters a speed-scaled stop zone ahead of the vessel. All
// reaction policy lives in the helm core.
class CollisionMonitor {
public:
	CollisionMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
		pnh.param<std::string>("/robot_frame", robot_frame_, "base_link");
		pnh.param("vessel_halfwidth", halfwidth_, 1.0);
		pnh.param("margin", margin_, 2.0);
		pnh.param("braking_horizon", horizon_, 3.0);
		pnh.param("min_points", min_points_, 2);
		pnh.param("heartbeat_period", heartbeat_period_, 1.0);

		cloud_sub_ = nh.subscribe("/obstacle_cloud/add", 5, &CollisionMonitor::cloudCallback, this);
		cmd_sub_ = nh.subscribe("/cmd_vel", 5, &CollisionMonitor::cmdCallback, this);
		status_pub_ = nh.advertise<tinyhelm_core::MonitorStatus>("/tinyhelm/monitor/collision", 5);
		heartbeat_timer_ = nh.createTimer(ros::Duration(heartbeat_period_), &CollisionMonitor::heartbeat, this);
		ROS_INFO("collision_monitor: halfwidth %.1fm, margin %.1fm, horizon %.1fs", halfwidth_, margin_, horizon_);
	}

private:
	void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) { speed_ = std::max(0.0, msg->linear.x); }

	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		sensor_msgs::PointCloud2 cloud;
		try {
			auto tf = tf_buffer_.lookupTransform(robot_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
			tf2::doTransform(*msg, cloud, tf);
		} catch (tf2::TransformException& e) {
			ROS_WARN_THROTTLE(5.0, "collision_monitor: transform failed: %s", e.what());
			return;
		}

		double zone_length = margin_ + speed_ * horizon_;
		int intruders = 0;
		sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y");
		for (; ix != ix.end(); ++ix, ++iy) {
			if (*ix > 0.0f && *ix < zone_length && std::fabs(*iy) < halfwidth_) intruders++;
		}

		if (intruders >= min_points_) {
			publish(tinyhelm_core::MonitorStatus::EMERGENCY, "COLLISION_IMMINENT", std::to_string(intruders), "Obstacle inside stop zone.");
			last_emergency_ = ros::Time::now();
		}
	}

	void heartbeat(const ros::TimerEvent&) {
		if ((ros::Time::now() - last_emergency_).toSec() > heartbeat_period_) publish(tinyhelm_core::MonitorStatus::OK, "CLEAR", "", "Stop zone clear.");
	}

	void publish(uint8_t level, const std::string& code, const std::string& data, const std::string& message) {
		tinyhelm_core::MonitorStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.name = "collision";
		msg.level = level;
		msg.code = code;
		msg.data = data;
		msg.message = message;
		status_pub_.publish(msg);
	}

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	ros::Subscriber cloud_sub_, cmd_sub_;
	ros::Publisher status_pub_;
	ros::Timer heartbeat_timer_;
	std::string robot_frame_;
	double halfwidth_, margin_, horizon_, heartbeat_period_;
	double speed_ = 0.0;
	int min_points_;
	ros::Time last_emergency_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "collision_monitor");
	ros::NodeHandle nh, pnh("~");
	CollisionMonitor node(nh, pnh);
	ros::spin();
	return 0;
}
