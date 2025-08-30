#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tinyhelm_core/TopicList.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <boost/bind.hpp>

class CmdVelMux {
public:
    CmdVelMux(ros::NodeHandle& nh) : nh(nh) {
        nh.param("teleop_timeout", teleop_timeout, 3.0);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        selector_sub = nh.subscribe("/mux_node/_active_topic", 10, &CmdVelMux::selector_callback, this);
        teleop_sub = nh.subscribe("/cmd_vel_teleop", 10, &CmdVelMux::teleop_callback, this);
        nav_topics_srv = nh.advertiseService("/mux_node/_nav_cmd_vel_array", &CmdVelMux::nav_topic_callback, this);
    }

private:
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub;
	ros::Subscriber teleop_sub;
    ros::Subscriber selector_sub;

    ros::ServiceServer override_topics_srv;
    ros::ServiceServer nav_topics_srv;

    std::unordered_map<std::string, ros::Subscriber> nav_subs;
    std::string selector;

    geometry_msgs::Twist last_override_msg;
    ros::Time last_teleop_time;
    double teleop_timeout;

    void selector_callback(const std_msgs::String::ConstPtr& msg) {
        selector = msg->data;
    }

    void teleop_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        cmd_vel_pub.publish(msg);
        last_teleop_time = ros::Time::now();
    }

    void cmd_vel_nav_callback(const geometry_msgs::Twist::ConstPtr& msg, const std::string& topic_name) {
        ros::Time now = ros::Time::now();
        double delta = (now - last_teleop_time).toSec();
        if (delta < teleop_timeout) {
            return;
        }
        if (selector == topic_name) {
            cmd_vel_pub.publish(msg);
        }
    }

    bool nav_topic_callback(tinyhelm_core::TopicList::Request& req, tinyhelm_core::TopicList::Response& res) {
        nav_subs.clear();
        for (const auto& topic : req.topics) {
            nav_subs[topic] = nh.subscribe<geometry_msgs::Twist>(
                topic,
                10,
                boost::bind(&CmdVelMux::cmd_vel_nav_callback, this, _1, topic)
            );
            ROS_INFO("Multiplexer subscribed to nav topic: %s", topic.c_str());
        }
        res.success = true;
        res.message = "Nav topics updated.";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mux_node");
    ros::NodeHandle nh("~");
    CmdVelMux mux(nh);
    ros::spin();
    return 0;
}