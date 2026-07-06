#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <tinyhelm_core/MonitorStatus.h>

#include <tinyhelm_obstacles/decay_grid.h>
#include <tinyhelm_obstacles/cost_field.h>
#include <tinyhelm_obstacles/theta_star.h>

#include <map>
#include <memory>

using namespace tinyhelm;

// Observes the strategic mission (/tinyhelm/mission) and the line planner's remaining
// tactical path (/line_planner/plan), maintains a two-layer decaying obstacle grid, and when
// the tactical corridor is intruded plans a detour with Theta* through the remaining
// strategic waypoints. It never commands anything: it publishes a proposed path and a
// MonitorStatus, and the helm core decides what to do with them.
class ObstaclePlannerNode {
public:
	ObstaclePlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
		pnh.param<std::string>("/planning_frame", planning_frame_, "local");
		pnh.param<std::string>("/robot_frame", robot_frame_, "base_link");
		
		pnh.param<std::string>("tactical_plan_topic", tactical_plan_topic_, "/waypoints/_plan");
		pnh.param<std::string>("divergence_param", divergence_param_, "/tinyhelm_waypoints_node/max_line_divergence");

		pnh.param("fine_resolution", fine_res_, 0.25);
		pnh.param("fine_size", fine_size_, 512);
		pnh.param("coarse_resolution", coarse_res_, 1.0);
		pnh.param("coarse_size", coarse_size_, 1024);

		pnh.param("hit_delta", hit_delta_, 32);
		pnh.param("miss_delta", miss_delta_, 12);
		pnh.param("occupied_threshold", occ_thresh_, 48);
		pnh.param("decay_half_life", half_life_, 60.0);

		pnh.param("inflate_radius", inflate_radius_, 1.5);
		pnh.param("soft_radius", soft_radius_, 5.0);
		pnh.param("soft_weight", soft_weight_, 2.0);

		pnh.param("min_detour", min_detour_, 25.0);
		pnh.param("detour_leg_fraction", detour_frac_, 0.5);
		pnh.param("max_detour", max_detour_, 200.0);
		pnh.param("budget_factor", budget_k_, 3.0);

		pnh.param("monitor_rate", monitor_rate_, 2.0);
		pnh.param("unreachable_cycles", unreachable_cycles_, 3);
		pnh.param("waypoint_reached_radius", waypoint_reached_radius_, 6.0);
		pnh.param("replan_cooldown", replan_cooldown_, 2.0);
		pnh.param("pose_jump_threshold", pose_jump_threshold_, 10.0);
		pnh.param("grid_publish_period", grid_publish_period_, 1.0);

		fine_ = std::make_unique<DecayGrid>(fine_res_, fine_size_, (int8_t)hit_delta_, (int8_t)miss_delta_, (int8_t)occ_thresh_, half_life_);
		coarse_ = std::make_unique<DecayGrid>(coarse_res_, coarse_size_, (int8_t)hit_delta_, (int8_t)miss_delta_, (int8_t)occ_thresh_, half_life_ * 2.0);

		hits_sub_ = nh.subscribe("/obstacle_cloud/add", 5, &ObstaclePlannerNode::hitsCallback, this);
		clear_sub_ = nh.subscribe("/obstacle_cloud/clear", 5, &ObstaclePlannerNode::clearCellsCallback, this);
		static_poly_sub_ = nh.subscribe("/obstacle_grid/add_polygon", 5, &ObstaclePlannerNode::staticPolygonCallback, this);
		grid_clear_sub_ = nh.subscribe("/obstacle_grid/clear", 1, &ObstaclePlannerNode::gridClearCallback, this);
		mission_sub_ = nh.subscribe("/tinyhelm/mission", 1, &ObstaclePlannerNode::missionCallback, this);
		tactical_sub_ = nh.subscribe(tactical_plan_topic_, 1, &ObstaclePlannerNode::tacticalCallback, this);

		path_pub_ = nh.advertise<nav_msgs::Path>("/obstacle_planner/path", 1, true);
		remaining_pub_ = nh.advertise<nav_msgs::Path>("/obstacle_planner/remaining", 1, true);
		status_pub_ = nh.advertise<tinyhelm_core::MonitorStatus>("/tinyhelm/monitor/obstacles", 5, true);
		fine_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/obstacle_planner/grid_fine", 1, true);
		coarse_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/obstacle_planner/grid_coarse", 1, true);

		timer_ = nh.createTimer(ros::Duration(1.0 / monitor_rate_), &ObstaclePlannerNode::tick, this);
		last_grid_publish_ = ros::Time(0);
		last_replan_ = ros::Time(0);
		ROS_INFO("obstacle_planner: fine %.2fm x %d, coarse %.2fm x %d, frame %s", fine_res_, fine_size_, coarse_res_, coarse_size_, planning_frame_.c_str());
	}

private:
	void hitsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) { ingest(msg, true); }
	void clearCellsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) { ingest(msg, false); }

	void ingest(const sensor_msgs::PointCloud2::ConstPtr& msg, bool hits) {
		sensor_msgs::PointCloud2 cloud;
		if (msg->header.frame_id != planning_frame_) {
			try {
				auto tf = tf_buffer_.lookupTransform(planning_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
				tf2::doTransform(*msg, cloud, tf);
			} catch (tf2::TransformException& e) {
				ROS_WARN_THROTTLE(5.0, "obstacle_planner: cloud transform failed: %s", e.what());
				return;
			}
		} else {
			cloud = *msg;
		}

		sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y");
		for (; ix != ix.end(); ++ix, ++iy) {
			if (hits) {
				fine_->addHit(*ix, *iy);
				coarse_->addHit(*ix, *iy);
			} else {
				fine_->addMiss(*ix, *iy);
				coarse_->addMiss(*ix, *iy);
			}
		}
		grids_dirty_ = true;
	}

	void staticPolygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
		if (msg->polygon.points.size() < 3) return;
		rasterizeStatic(*fine_, msg->polygon);
		rasterizeStatic(*coarse_, msg->polygon);
		grids_dirty_ = true;
	}

	void rasterizeStatic(DecayGrid& grid, const geometry_msgs::Polygon& poly) {
		double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
		for (const auto& p : poly.points) {
			minx = std::min(minx, (double)p.x); maxx = std::max(maxx, (double)p.x);
			miny = std::min(miny, (double)p.y); maxy = std::max(maxy, (double)p.y);
		}
		for (double y = miny; y <= maxy; y += grid.resolution()) {
			for (double x = minx; x <= maxx; x += grid.resolution()) {
				if (pointInPolygon(x, y, poly)) grid.setStatic(x, y, true);
			}
		}
	}

	bool pointInPolygon(double x, double y, const geometry_msgs::Polygon& poly) {
		bool inside = false;
		size_t n = poly.points.size();
		for (size_t i = 0, j = n - 1; i < n; j = i++) {
			double xi = poly.points[i].x, yi = poly.points[i].y, xj = poly.points[j].x, yj = poly.points[j].y;
			if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
		}
		return inside;
	}

	void gridClearCallback(const std_msgs::Empty::ConstPtr&) {
		fine_->clear();
		coarse_->clear();
		grids_dirty_ = true;
		ROS_INFO("obstacle_planner: grids cleared by request");
	}

	void missionCallback(const nav_msgs::Path::ConstPtr& msg) {
		mission_ = *msg;
		next_wp_index_ = 0;
		unreachable_counts_.clear();
		last_published_.clear();
		blocked_ = false;
		if (mission_.poses.empty()) publishStatus(tinyhelm_core::MonitorStatus::OK, "CLEAR", "", "No active mission.");
	}

	void tacticalCallback(const nav_msgs::Path::ConstPtr& msg) { tactical_ = *msg; }

	// Strategic waypoints are always visited in order (detours never skip them), so passing
	// within reach radius of the next one is the ground truth for progress. Matching against
	// the tactical path by coordinates breaks on and_return/loiter missions where the same
	// coordinates appear twice.
	void updateRemaining(double rx, double ry) {
		while (next_wp_index_ < mission_.poses.size()) {
			const auto& p = mission_.poses[next_wp_index_].pose.position;
			if (std::hypot(p.x - rx, p.y - ry) > waypoint_reached_radius_) break;
			next_wp_index_++;
		}
		remaining_.assign(mission_.poses.begin() + next_wp_index_, mission_.poses.end());
	}

	bool getRobotPose(double& x, double& y) {
		try {
			auto tf = tf_buffer_.lookupTransform(planning_frame_, robot_frame_, ros::Time(0));
			x = tf.transform.translation.x;
			y = tf.transform.translation.y;
			return true;
		} catch (tf2::TransformException&) {
			return false;
		}
	}

	void tick(const ros::TimerEvent& ev) {
		double dt = last_tick_.isZero() ? 1.0 / monitor_rate_ : (ev.current_real - last_tick_).toSec();
		last_tick_ = ev.current_real;

		double rx, ry;
		if (!getRobotPose(rx, ry)) return;

		if (have_last_pose_ && std::hypot(rx - last_rx_, ry - last_ry_) > pose_jump_threshold_) {
			ROS_WARN("obstacle_planner: pose jumped %.1fm, assuming ENU reset and clearing grids", std::hypot(rx - last_rx_, ry - last_ry_));
			fine_->clear();
			coarse_->clear();
			grids_dirty_ = true;
		}
		last_rx_ = rx;
		last_ry_ = ry;
		have_last_pose_ = true;

		fine_->recenter(rx, ry, coarse_.get());
		coarse_->recenter(rx, ry, nullptr);
		fine_->decay(dt);
		coarse_->decay(dt);

		if ((ev.current_real - last_grid_publish_).toSec() >= grid_publish_period_) {
			publishGrid(*fine_, fine_grid_pub_);
			publishGrid(*coarse_, coarse_grid_pub_);
			last_grid_publish_ = ev.current_real;
		}

		if (mission_.poses.empty()) return;
		updateRemaining(rx, ry);
		if (remaining_.empty()) return;

		nav_msgs::Path rem;
		rem.header.frame_id = planning_frame_;
		rem.header.stamp = ev.current_real;
		rem.poses = remaining_;
		remaining_pub_.publish(rem);

		double corridor = 0.0;
		ros::param::getCached(divergence_param_, corridor);
		effective_inflate_ = inflate_radius_ + corridor;

		std::vector<Capsule> fence = buildGeofence(rx, ry);
		fine_field_.build(*fine_, effective_inflate_, soft_radius_, soft_weight_, fence);
		coarse_field_.build(*coarse_, effective_inflate_, soft_radius_, soft_weight_, fence);
		grids_dirty_ = false;

		bool path_clear = tactical_.poses.size() >= 2 && corridorClear();

		if (path_clear) {
			if (blocked_) ROS_INFO("obstacle_planner: corridor clear again");
			blocked_ = false;
			publishStatus(tinyhelm_core::MonitorStatus::OK, "CLEAR", "", "Corridor clear.");
			return;
		}

		blocked_ = true;
		if ((ev.current_real - last_replan_).toSec() < replan_cooldown_) return;
		last_replan_ = ev.current_real;
		replan(rx, ry);
	}

	std::vector<Capsule> buildGeofence(double rx, double ry) {
		std::vector<Capsule> fence;
		double px = rx, py = ry;
		for (const auto& wp : remaining_) {
			double wx = wp.pose.position.x, wy = wp.pose.position.y;
			double leg = std::hypot(wx - px, wy - py);
			double radius = std::min(max_detour_, std::max(min_detour_, leg * detour_frac_));
			fence.push_back({px, py, wx, wy, radius});
			px = wx;
			py = wy;
		}
		return fence;
	}

	bool corridorClear() {
		for (size_t i = 1; i < tactical_.poses.size(); i++) {
			double x1 = tactical_.poses[i - 1].pose.position.x, y1 = tactical_.poses[i - 1].pose.position.y;
			double x2 = tactical_.poses[i].pose.position.x, y2 = tactical_.poses[i].pose.position.y;
			double len = std::hypot(x2 - x1, y2 - y1);
			int steps = std::max(1, (int)(len / (fine_res_ * 2.0)));
			for (int s = 0; s <= steps; s++) {
				double t = (double)s / steps;
				double x = x1 + t * (x2 - x1), y = y1 + t * (y2 - y1);
				int cx, cy;
				if (fine_field_.worldToCell(x, y, cx, cy)) {
					if (fine_field_.obstacleDistance(cx, cy) < effective_inflate_ - fine_res_) return false;
				} else if (coarse_field_.obstacleDistanceAt(x, y) < effective_inflate_ - coarse_res_) {
					return false;
				}
			}
		}
		return true;
	}

	bool pointLethal(double x, double y) {
		int cx, cy;
		if (fine_field_.worldToCell(x, y, cx, cy)) return fine_field_.lethal(cx, cy);
		return coarse_field_.lethalAt(x, y);
	}

	bool lineClear(const PlanPoint& a, const PlanPoint& b) {
		double len = std::hypot(b.x - a.x, b.y - a.y);
		int steps = std::max(1, (int)(len / (fine_res_ * 2.0)));
		for (int s = 1; s < steps; s++) {
			double t = (double)s / steps;
			if (pointLethal(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))) return false;
		}
		return true;
	}

	// Theta* style string pulling over a stitched leg: greedily link each point to the
	// furthest point it has line of sight to, dropping everything in between. Fixes the
	// fine/coarse splice and coarse grid discretization into one taut segment chain.
	std::vector<PlanPoint> smoothLeg(const std::vector<PlanPoint>& leg) {
		if (leg.size() <= 2) return leg;
		std::vector<PlanPoint> out;
		size_t i = 0;
		out.push_back(leg[0]);
		while (i < leg.size() - 1) {
			size_t j = leg.size() - 1;
			while (j > i + 1 && !lineClear(leg[i], leg[j])) j--;
			out.push_back(leg[j]);
			i = j;
		}
		return out;
	}

	bool pathsSimilar(const std::vector<geometry_msgs::PoseStamped>& a, const std::vector<geometry_msgs::PoseStamped>& b) {
		if (a.empty() || b.empty()) return false;
		auto dev = [](const std::vector<geometry_msgs::PoseStamped>& p, const std::vector<geometry_msgs::PoseStamped>& q) {
			double worst = 0.0;
			for (const auto& pp : p) {
				double best = 1e18;
				for (size_t i = 1; i < q.size(); i++) {
					Capsule seg{q[i - 1].pose.position.x, q[i - 1].pose.position.y, q[i].pose.position.x, q[i].pose.position.y, 0.0};
					best = std::min(best, seg.distance(pp.pose.position.x, pp.pose.position.y));
				}
				worst = std::max(worst, best);
			}
			return worst;
		};
		return dev(a, b) < 2.0 && dev(b, a) < 2.0;
	}

	void replan(double rx, double ry) {
		nav_msgs::Path out;
		out.header.frame_id = planning_frame_;
		out.header.stamp = ros::Time::now();

		double px = rx, py = ry;
		bool beyond_horizon = false;
		for (size_t wi = 0; wi < remaining_.size(); wi++) {
			double gx = remaining_[wi].pose.position.x, gy = remaining_[wi].pose.position.y;

			if (beyond_horizon) {
				out.poses.push_back(remaining_[wi]);
				continue;
			}

			double clip_x = gx, clip_y = gy;
			bool clipped = clipToField(coarse_field_, px, py, clip_x, clip_y);

			if (!clipped && coarse_field_.lethalAt(gx, gy)) {
				if (bumpUnreachable(wi)) return;
				publishStatus(tinyhelm_core::MonitorStatus::BLOCKED, "PATH_BLOCKED", std::to_string(strategicIndex(wi)), "Waypoint occupied, confirming...");
				return;
			}

			std::vector<PlanPoint> leg = planLeg(px, py, clip_x, clip_y);
			if (leg.empty()) {
				if (bumpUnreachable(wi)) return;
				publishStatus(tinyhelm_core::MonitorStatus::BLOCKED, "PATH_BLOCKED", std::to_string(strategicIndex(wi)), "No path within geofence.");
				return;
			}

			double direct = std::hypot(clip_x - px, clip_y - py);
			if (direct > coarse_res_ && ThetaStar::pathLength(leg) > budget_k_ * direct) {
				if (bumpUnreachable(wi)) return;
				publishStatus(tinyhelm_core::MonitorStatus::BLOCKED, "PATH_BLOCKED", std::to_string(strategicIndex(wi)), "Detour exceeds budget.");
				return;
			}

			leg = smoothLeg(leg);
			for (size_t i = (out.poses.empty() ? 0 : 1); i < leg.size(); i++) out.poses.push_back(makePose(leg[i].x, leg[i].y, remaining_[wi].pose.position.z));

			if (clipped) {
				beyond_horizon = true;
				out.poses.push_back(remaining_[wi]);
			} else {
				out.poses.back() = remaining_[wi];
				unreachable_counts_.erase(strategicIndex(wi));
			}
			px = gx;
			py = gy;
		}

		if (pathsSimilar(out.poses, last_published_)) {
			publishStatus(tinyhelm_core::MonitorStatus::WARN, "REPLAN_READY", "", "Detour available (unchanged).");
			return;
		}
		last_published_ = out.poses;
		path_pub_.publish(out);
		publishStatus(tinyhelm_core::MonitorStatus::WARN, "REPLAN_READY", "", "Detour planned around obstacles.");
		ROS_INFO("obstacle_planner: replanned %zu poses through %zu waypoints", out.poses.size(), remaining_.size());
	}

	// Plan on the coarse field, then re-refine the portion inside the fine extent on the fine field
	std::vector<PlanPoint> planLeg(double sx, double sy, double gx, double gy) {
		std::vector<PlanPoint> coarse_path = planner_.plan(coarse_field_, sx, sy, gx, gy);
		if (coarse_path.empty()) return coarse_path;

		size_t split = 0;
		for (size_t i = 0; i < coarse_path.size(); i++) {
			int cx, cy;
			if (!fine_field_.worldToCell(coarse_path[i].x, coarse_path[i].y, cx, cy)) break;
			split = i;
		}
		if (split < 1) return coarse_path;

		std::vector<PlanPoint> fine_path = planner_.plan(fine_field_, sx, sy, coarse_path[split].x, coarse_path[split].y);
		if (fine_path.empty()) return coarse_path;

		fine_path.insert(fine_path.end(), coarse_path.begin() + split + 1, coarse_path.end());
		return fine_path;
	}

	// If the goal lies outside the coarse field, pull it back along the segment to just inside;
	// beyond the horizon we assume free water and pass the remaining waypoints through untouched
	bool clipToField(const CostField& field, double sx, double sy, double& gx, double& gy) {
		int cx, cy;
		if (field.worldToCell(gx, gy, cx, cy)) return false;
		double lo = 0.0, hi = 1.0;
		for (int i = 0; i < 24; i++) {
			double mid = 0.5 * (lo + hi);
			double x = sx + mid * (gx - sx), y = sy + mid * (gy - sy);
			if (field.worldToCell(x, y, cx, cy)) lo = mid; else hi = mid;
		}
		double t = std::max(0.0, lo - 0.02);
		gx = sx + t * (gx - sx);
		gy = sy + t * (gy - sy);
		return true;
	}

	int strategicIndex(size_t remaining_index) { return (int)(next_wp_index_ + remaining_index); }

	// Returns true once the failure has persisted long enough to declare the waypoint unreachable
	bool bumpUnreachable(size_t remaining_index) {
		int si = strategicIndex(remaining_index);
		if (++unreachable_counts_[si] < unreachable_cycles_) return false;
		publishStatus(tinyhelm_core::MonitorStatus::BLOCKED, "WAYPOINT_UNREACHABLE", std::to_string(si), "Waypoint unreachable after repeated attempts.");
		return true;
	}

	geometry_msgs::PoseStamped makePose(double x, double y, double z) {
		geometry_msgs::PoseStamped p;
		p.header.frame_id = planning_frame_;
		p.pose.position.x = x;
		p.pose.position.y = y;
		p.pose.position.z = z;
		p.pose.orientation.w = 1.0;
		return p;
	}

	void publishStatus(uint8_t level, const std::string& code, const std::string& data, const std::string& message) {
		tinyhelm_core::MonitorStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.name = "obstacles";
		msg.level = level;
		msg.code = code;
		msg.data = data;
		msg.mission_stamp = mission_.header.stamp;
		msg.message = message;
		status_pub_.publish(msg);
	}

	void publishGrid(const DecayGrid& grid, ros::Publisher& pub) {
		if (pub.getNumSubscribers() == 0) return;
		nav_msgs::OccupancyGrid msg;
		msg.header.frame_id = planning_frame_;
		msg.header.stamp = ros::Time::now();
		msg.info.resolution = grid.resolution();
		msg.info.width = msg.info.height = grid.size();
		msg.info.origin.position.x = grid.originX();
		msg.info.origin.position.y = grid.originY();
		msg.info.origin.orientation.w = 1.0;
		msg.data.resize(grid.size() * grid.size());
		for (int y = 0; y < grid.size(); y++) {
			for (int x = 0; x < grid.size(); x++) {
				int8_t v = grid.at(x, y);
				int8_t& out = msg.data[y * grid.size() + x];
				if (grid.isStatic(x, y) || v >= grid.occupiedThreshold()) out = 100;
				else if (v > 0) out = (int8_t)std::max(1, std::min(98, (int)v * 98 / grid.occupiedThreshold()));
				else if (v < 0) out = 0;
				else out = -1;
			}
		}
		pub.publish(msg);
	}

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	ros::Subscriber hits_sub_, clear_sub_, static_poly_sub_, grid_clear_sub_, mission_sub_, tactical_sub_;
	ros::Publisher path_pub_, remaining_pub_, status_pub_, fine_grid_pub_, coarse_grid_pub_;
	ros::Timer timer_;

	std::string planning_frame_, robot_frame_, tactical_plan_topic_, divergence_param_;
	double fine_res_, coarse_res_;
	int fine_size_, coarse_size_;
	int hit_delta_, miss_delta_, occ_thresh_;
	double half_life_, inflate_radius_, soft_radius_, soft_weight_, effective_inflate_ = 0.0;
	std::vector<geometry_msgs::PoseStamped> last_published_;
	double min_detour_, detour_frac_, max_detour_, budget_k_;
	double monitor_rate_, replan_cooldown_, pose_jump_threshold_, grid_publish_period_;
	int unreachable_cycles_;

	std::unique_ptr<DecayGrid> fine_, coarse_;
	CostField fine_field_, coarse_field_;
	ThetaStar planner_;

	nav_msgs::Path mission_, tactical_;
	std::vector<geometry_msgs::PoseStamped> remaining_;
	size_t next_wp_index_ = 0;
	double waypoint_reached_radius_ = 6.0;
	std::map<int, int> unreachable_counts_;
	bool blocked_ = false, grids_dirty_ = false, have_last_pose_ = false;
	double last_rx_ = 0.0, last_ry_ = 0.0;
	ros::Time last_tick_, last_grid_publish_, last_replan_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "obstacle_planner_node");
	ros::NodeHandle nh, pnh("~");
	ObstaclePlannerNode node(nh, pnh);
	ros::spin();
	return 0;
}
