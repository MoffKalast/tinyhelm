#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <unordered_set>

// Hash function for 3D points (for deduplication)
struct Point3DHash {
    std::size_t operator()(const std::tuple<int, int, int>& p) const {
        return std::hash<int>()(std::get<0>(p)) ^
               (std::hash<int>()(std::get<1>(p)) << 1) ^
               (std::hash<int>()(std::get<2>(p)) << 2);
    }
};

class ScanToCloudNode {
private:
    ros::Subscriber scan_sub_;
    ros::Publisher hit_cells_pub_;
    ros::Publisher clear_cells_pub_;
    
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::string fixed_frame_;
    std::string scan_topic_;
    
    double cell_resolution_;
    double max_range_;
    bool enable_raycasting_;
    
public:
    ScanToCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : tf_buffer_(), tf_listener_(tf_buffer_) {
        
        pnh.param<std::string>("fixed_frame", fixed_frame_, "local");
        pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh.param<double>("cell_resolution", cell_resolution_, 0.2);
        pnh.param<double>("max_range", max_range_, 10.0);
        pnh.param<bool>("enable_raycasting", enable_raycasting_, true);
        
        scan_sub_ = nh.subscribe(scan_topic_, 10, &ScanToCloudNode::scanCallback, this);
        hit_cells_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud/add", 10);
        clear_cells_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud/clear", 10);
        
        ROS_INFO("scan_to_cloud node started:");
        ROS_INFO("  Listening to: %s", scan_topic_.c_str());
        ROS_INFO("  Publishing hit cells to: /obstacle_cloud/add");
        ROS_INFO("  Publishing clear cells to: /obstacle_cloud/clear");
        ROS_INFO("  Fixed frame: %s", fixed_frame_.c_str());
        ROS_INFO("  Cell resolution: %.2fm", cell_resolution_);
        ROS_INFO("  Raycasting enabled: %s", enable_raycasting_ ? "true" : "false");
    }

private:
    std::tuple<int, int, int> quantizeToGrid(double x, double y, double z) {
        return std::make_tuple(
            static_cast<int>(std::round(x / cell_resolution_)),
            static_cast<int>(std::round(y / cell_resolution_)),
            static_cast<int>(std::round(z / cell_resolution_))
        );
    }
    
    std::vector<std::tuple<int, int, int>> raycastToCells(
        double start_x, double start_y, double start_z,
        double end_x, double end_y, double end_z) {
        
        std::vector<std::tuple<int, int, int>> cells;
        
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double dz = end_z - start_z;
        double length = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (length < cell_resolution_) {
            return cells;
        }
        
        dx /= length;
        dy /= length;
        dz /= length;
        
        double step_size = cell_resolution_ * 0.5;
        int num_steps = static_cast<int>(length / step_size);
        
        std::unordered_set<std::tuple<int, int, int>, Point3DHash> unique_cells;
        
        // Sample points along the ray (excluding the final hit point)
        for (int i = 1; i < num_steps; ++i) {
            double t = i * step_size;
            double x = start_x + t * dx;
            double y = start_y + t * dy;
            double z = start_z + t * dz;
            
            auto cell = quantizeToGrid(x, y, z);
            unique_cells.insert(cell);
        }
        
        cells.reserve(unique_cells.size());
        for (const auto& cell : unique_cells) {
            cells.push_back(cell);
        }
        
        return cells;
    }
    
    sensor_msgs::PointCloud2 createPointCloud(
        const std::unordered_set<std::tuple<int, int, int>, Point3DHash>& cells,
        const std_msgs::Header& header) {
        
        sensor_msgs::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;
        cloud.width = cells.size();
        cloud.fields.resize(3);
        
        // Set up field descriptions
        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[0].count = 1;
        
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[1].count = 1;
        
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 8;
        cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[2].count = 1;
        
        cloud.point_step = 12;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step);
        
        // Fill in the data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        
        for (const auto& cell : cells) {
            *iter_x = std::get<0>(cell) * cell_resolution_;
            *iter_y = std::get<1>(cell) * cell_resolution_;
            *iter_z = std::get<2>(cell) * cell_resolution_;
            ++iter_x; ++iter_y; ++iter_z;
        }
        
        return cloud;
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        try {
            std::unordered_set<std::tuple<int, int, int>, Point3DHash> hit_cells_set;
            std::unordered_set<std::tuple<int, int, int>, Point3DHash> clear_cells_set;
            
            if (enable_raycasting_) {
                // Process laser scan for raycasting
                for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
                    float range = scan_msg->ranges[i];
                    
                    if (std::isnan(range) || std::isinf(range) || 
                        range < scan_msg->range_min || range > scan_msg->range_max ||
                        range > max_range_) {
                        continue;
                    }
                    
                    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                    double end_x = range * std::cos(angle);
                    double end_y = range * std::sin(angle);
                    double end_z = 0.0;
                    
                    // Add hit cell
                    auto hit_cell = quantizeToGrid(end_x, end_y, end_z);
                    hit_cells_set.insert(hit_cell);
                    
                    // Raycast for clear cells
                    auto ray_cells = raycastToCells(0.0, 0.0, 0.0, end_x, end_y, end_z);
                    
                    for (const auto& cell : ray_cells) {
                        if (hit_cells_set.find(cell) == hit_cells_set.end()) {
                            clear_cells_set.insert(cell);
                        }
                    }
                }
            } else {
                // Just process hit cells without raycasting
                sensor_msgs::PointCloud2 cloud_sensor_frame;
                projector_.projectLaser(*scan_msg, cloud_sensor_frame);
                
                sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_sensor_frame, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_sensor_frame, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_sensor_frame, "z");
                
                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                        continue;
                    }
                    
                    auto hit_cell = quantizeToGrid(*iter_x, *iter_y, *iter_z);
                    hit_cells_set.insert(hit_cell);
                }
            }
            
            // Transform and publish hit cells
            if (!hit_cells_set.empty()) {
                std_msgs::Header header;
                header.stamp = scan_msg->header.stamp;
                header.frame_id = scan_msg->header.frame_id;
                
                sensor_msgs::PointCloud2 hit_cells_cloud = createPointCloud(hit_cells_set, header);
                
                // Transform to fixed frame
                sensor_msgs::PointCloud2 hit_cells_transformed;
                tf_buffer_.transform(hit_cells_cloud, hit_cells_transformed, fixed_frame_, ros::Duration(0.1));
                
                hit_cells_pub_.publish(hit_cells_transformed);
            }
            
            // Transform and publish clear cells
            if (!clear_cells_set.empty()) {
                std_msgs::Header header;
                header.stamp = scan_msg->header.stamp;
                header.frame_id = scan_msg->header.frame_id;
                
                sensor_msgs::PointCloud2 clear_cells_cloud = createPointCloud(clear_cells_set, header);
                
                sensor_msgs::PointCloud2 clear_cells_transformed;
                tf_buffer_.transform(clear_cells_cloud, clear_cells_transformed, fixed_frame_, ros::Duration(0.1));
                
                clear_cells_pub_.publish(clear_cells_transformed);
            }
            
            ROS_DEBUG("Processed %zu hit cells, %zu clear cells", 
                     hit_cells_set.size(), clear_cells_set.size());
            
        } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM_THROTTLE(1.0, "Transform failed: " << ex.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_to_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ScanToCloudNode node(nh, pnh);
    ros::spin();
    
    return 0;
}