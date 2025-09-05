#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <unordered_map>
#include <string>
#include <boost/bind.hpp>

class CmdVelMux {
public:
	CmdVelMux(ros::NodeHandle& nh) : nh(nh) {
		ros::NodeHandle nh_mux("tinyhelm_core/cmd_vel_mux");

		nh_mux.param("teleop_timeout", teleop_timeout, 3.0);
		nh_mux.param<std::string>("out_topic", out_topic, std::string("/cmd_vel"));
		nh_mux.param<std::string>("teleop_topic", teleop_topic, std::string("/cmd_vel_teleop"));
		nh_mux.param<std::string>("selector_topic", selector_topic, std::string("/cmd_vel_mux/_active_topic"));
		nh_mux.param<std::string>("teleop_override_active_topic", teleop_override_active_topic, std::string("/teleop_override_active"));

		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(out_topic, 10);
		teleop_override_pub = nh.advertise<std_msgs::Bool>(teleop_override_active_topic, 10, true);

		selector_sub = nh.subscribe(selector_topic, 10, &CmdVelMux::selector_callback, this);
		teleop_sub = nh.subscribe(teleop_topic, 10, &CmdVelMux::teleop_callback, this);

		ROS_INFO("CmdVelMux out topic: %s", out_topic.c_str());
		ROS_INFO("CmdVelMux teleop override: %s", teleop_topic.c_str());

		//compile all the controller cmd_vels we need to multiplex
		std::vector<std::string> all_params;
		ros::param::getParamNames(all_params);

		for (const auto& param_name : all_params) {
			if (param_name.find("/tinyhelm_core/controllers/") != 0)
				continue;
			
			if(param_name.find("/cmd_vel") == -1)
				continue;

			std::string topic;
			if (ros::param::get(param_name, topic)) {
				nav_subs[topic] = nh.subscribe<geometry_msgs::Twist>(
					topic,
					10,
					boost::bind(&CmdVelMux::cmd_vel_nav_callback, this, _1, topic)
				);
				ROS_INFO("CmdVelMux found: %s", topic.c_str());
			}
		}

		ROS_INFO_STREAM("CmdVelMux ready.");
	}


private:
	ros::NodeHandle nh;
	std::string out_topic;
	std::string teleop_topic;
	std::string selector_topic;
	std::string teleop_override_active_topic;
	double teleop_timeout;

	ros::Publisher cmd_vel_pub;
	ros::Publisher teleop_override_pub;
	ros::Subscriber teleop_sub;
	ros::Subscriber selector_sub;
	std::unordered_map<std::string, ros::Subscriber> nav_subs;

	std::string selector;
	ros::Time last_teleop_time;
	bool teleop_override;

	void selector_callback(const std_msgs::String::ConstPtr& msg) {
		selector = msg->data;
	}

	void teleop_callback(const geometry_msgs::Twist::ConstPtr& msg) {
		cmd_vel_pub.publish(msg);
		last_teleop_time = ros::Time::now();

		if(!teleop_override){
			teleop_override = true;
			publish_override_state();
		}
	}

	void cmd_vel_nav_callback(const geometry_msgs::Twist::ConstPtr& msg, const std::string& topic_name) {
		ros::Time now = ros::Time::now();
		double delta = (now - last_teleop_time).toSec();

		if (delta < teleop_timeout) {
			return;
		}

		if(teleop_override){
			teleop_override = false;
			publish_override_state();
		}

		if (selector == topic_name) {
			cmd_vel_pub.publish(msg);
		}
	}

	void publish_override_state() {
		std_msgs::Bool msg;
		msg.data = teleop_override;
		teleop_override_pub.publish(msg);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mux_node");
	ros::NodeHandle nh("~");
	CmdVelMux mux(nh);
	ros::spin();
	return 0;
}