#!/usr/bin/env python3
import math
import rospy
import noise
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from wavefield import WaveField
from geo_utils import integrate_latlon

class VesselSimNode:
    def __init__(self):

        self.param_base_link = rospy.get_param("~base_link", "base_link")
        self.param_imu_link = rospy.get_param("~imu_link", "imu_link")

        # Position state
        self.lat = math.radians(rospy.get_param("~origin_lat", 0.0))
        self.lon = math.radians(rospy.get_param("~origin_lon", 0.0))

        self.min_linear_vel = math.radians(rospy.get_param("~min_linear_vel", 0.2))
        self.min_angular_vel = math.radians(rospy.get_param("~min_angular_vel", 0.05))
        self.yaw = 0.0

        # Dynamics state (body frame velocities)
        self.u = 0.0  # surge
        self.v = 0.0  # sway
        self.r = 0.0  # yaw rate

        # Commanded velocities
        self.cmd_u = 0.0
        self.cmd_v = 0.0
        self.cmd_r = 0.0

        # Inertia params (seconds)
        self.tau_lin = rospy.get_param("~tau_lin", 0.5)
        self.tau_yaw = rospy.get_param("~tau_yaw", 0.25)

        # Simulation timing
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.realtime_factor = rospy.get_param("~realtime_factor", 1.0)

        # Environment params
        self.current_n = rospy.get_param("~current_n", 0.01)
        self.current_e = rospy.get_param("~current_e", 0.0)
        self.wind_base_n = rospy.get_param("~wind_base_n", 0.0)
        self.wind_base_e = rospy.get_param("~wind_base_e", -0.01)
        self.wind_amp = rospy.get_param("~wind_amp", 0.5)
        self.wind_freq = rospy.get_param("~wind_freq", 0.01)

        # Waves
        wave_cfg = {
            'wave_height': rospy.get_param("~wave_height", 2.0),
            'wave_scale': rospy.get_param("~wave_scale", 0.1),
            'time_scale': rospy.get_param("~wave_time_scale", 0.5),
            'octaves': rospy.get_param("~octaves", 4),
            'persistence': rospy.get_param("~persistence", 0.3),
            'lacunarity': rospy.get_param("~lacunarity", 2.0)
        }
        self.wavefield = WaveField(wave_cfg)

        # ROS I/O
        self.sub_cmd = rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=1)
        self.pub_fix = rospy.Publisher("/fix", NavSatFix, queue_size=1)
        self.pub_imu = rospy.Publisher("/imu/data", Imu, queue_size=1)

        self.sim_time = 0.0

        self.deadman_timer_stamp = -1

    def cmd_cb(self, msg):

        if abs(msg.linear.x) > 0 and abs(msg.linear.x) < self.min_linear_vel:
            msg.linear.x = math.copysign(self.min_linear_vel, msg.linear.x)

        if abs(msg.angular.z) > 0 and abs(msg.angular.z) < self.min_angular_vel:
            msg.angular.z = math.copysign(self.min_angular_vel, msg.angular.z)

        self.cmd_u = msg.linear.x
        self.cmd_v = msg.linear.y
        self.cmd_r = msg.angular.z
        self.deadman_timer_stamp = rospy.get_time()

    def step(self, dt):
        self.sim_time += dt

        if rospy.get_time() - self.deadman_timer_stamp > 0.2:
            self.cmd_u = 0
            self.cmd_v = 0
            self.cmd_r = 0

        # --- Inertia filter ---
        alpha_lin = dt / self.tau_lin
        alpha_yaw = dt / self.tau_yaw
        self.u += (self.cmd_u - self.u) * alpha_lin
        self.v += (self.cmd_v - self.v) * alpha_lin
        self.r += (self.cmd_r - self.r) * alpha_yaw

        # --- Body -> ENU velocities ---
        ve = self.u * math.cos(self.yaw) - self.v * math.sin(self.yaw)
        vn = self.u * math.sin(self.yaw) + self.v * math.cos(self.yaw)

        # --- Add current & wind ---
        wind_n = self.wind_base_n + self.wind_amp * noise.pnoise1(self.sim_time * self.wind_freq)
        wind_e = self.wind_base_e + self.wind_amp * noise.pnoise1((self.sim_time + 100) * self.wind_freq)

        vn_total = vn + self.current_n + wind_n
        ve_total = ve + self.current_e + wind_e

        # --- Integrate position ---
        self.lat, self.lon = integrate_latlon(self.lat, self.lon, vn_total, ve_total, dt)
        self.yaw += self.r * dt

        # --- Wavefield ---
        x_m = self.lat * 111000.0
        y_m = self.lon * 111000.0 * math.cos(self.lat)
        z = self.wavefield.get_wave_height(x_m, y_m, self.sim_time)
        pitch, roll = self.wavefield.get_wave_normal(x_m, y_m, self.sim_time)

        return z, pitch, roll

    def publish_data(self, z, pitch, roll):
        # NavSatFix
        fix = NavSatFix()
        fix.header.stamp = rospy.Time.now()
        fix.header.frame_id = self.param_base_link
        fix.latitude = math.degrees(self.lat)
        fix.longitude = math.degrees(self.lon)
        fix.altitude = z
        fix.status.status = 0
        fix.status.service = 1
        self.pub_fix.publish(fix)

        # IMU
        imu = Imu()
        imu.header.stamp = fix.header.stamp
        imu.header.frame_id = self.param_imu_link
        rotation = Rotation.from_euler('xyz', [roll, pitch, self.yaw])
        quat = rotation.as_quat()  # [x,y,z,w]
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = quat
        self.pub_imu.publish(imu)

    def run(self):
        dt = self.realtime_factor / self.rate_hz
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            z, pitch, roll = self.step(dt)
            self.publish_data(z, pitch, roll)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("vessel_sim")
    node = VesselSimNode()
    node.run()
