# The Test Sim

A minimal vessel simulator, enough to exercise the whole stack without hardware or Gazebo.

Two nodes: one integrates vessel dynamics and emits GNSS and IMU, the other raycasts a lidar against polygon obstacles. Both are a few hundred lines of Python and start instantly, which is the point. Testing a helm does not need buoyancy, hull meshes or a physics engine, it needs a vessel that responds to `/cmd_vel` slowly and gets pushed around by things it cannot control.

Currently hardcoded to a diff drive ASV (e.g. BlueBoat), TODO actual parametrization of vehicle dynamics.

Only `/gnss/fix` and `/imu/data` come out. Something has to fuse those into the `planning_frame` to `robot_frame` transform that every node in the stack looks up, as it would on the real vehicle.

## Obstacles

`sim_obstacles_node.py` raycasts against a list of polygons and publishes two LaserScans, `scan_verified` and `scan_filtered`, matching the two-stream input the obstacle package expects. Both carry gaussian range noise at `noise_sigma` and random dropouts at `dropout_probability`; the filtered stream additionally gets spurious returns at `speckle_rate`, which is what makes the reliable and unreliable split do any work. A pipeline that only ever sees clean scans will never demonstrate that it can tell chop from a buoy.

Polygons can be set as a rosparam at launch:

```xml
<rosparam param="polygons">[[[30.0, -10.0], [40.0, -10.0], [40.0, 10.0], [30.0, 10.0]]]</rosparam>
```

or added and removed at runtime through Vizanti by publishing a Polygon to `/sim/add_obstacle_polygon`.

Runtime addition is how the appearing-obstacle scenarios are built: start a mission, drop a polygon onto the line, watch the evidence accumulate and the replan fire.

## Launch

```bash
roslaunch tinyhelm_sim sim.launch
```

Arguments: `fixed_frame`, `origin_lat`, `origin_lon`, `realtime_factor`, `rate`. Raising `realtime_factor` runs long survey missions faster than real time, though the controllers still tick at their own rates so pushing it far will change how the loops behave.

## Params

Both nodes read private params, so these go in the launch file rather than a shared yaml.

`sim_node.py`:

```yaml
base_link: base_link
imu_link: imu_link

origin_lat: 43.379296
origin_lon: 16.602522

min_linear_vel: 0.2
min_angular_vel: 0.05

# First order lag time constants
tau_lin: 0.5
tau_yaw: 0.25

rate: 30.0
realtime_factor: 1.0

# Steady water current, m/s in the local frame
current_n: 0.01
current_e: 0.0

# Wind oscillates around the base vector
wind_base_n: 0.0
wind_base_e: -0.01
wind_amp: 0.5
wind_freq: 0.01

wave_height: 1.0
wave_scale: 0.05
wave_time_scale: 0.5
octaves: 4
persistence: 0.3
lacunarity: 2.0
```

`sim_obstacles_node.py`:

```yaml
fixed_frame: local
laser_frame: laser

num_beams: 360
max_range: 50.0
min_range: 0.5
rate: 10.0

noise_sigma: 0.05
dropout_probability: 0.02

# Only applied to the unreliable stream
speckle_rate: 1.0

polygons: []
```

## Subscribed Topics

- `/cmd_vel` (Twist), the output of the stack

- `/sim/add_obstacle_polygon` (PolygonStamped)

- `/sim/clear_obstacle_polygons` (Empty)

## Published Topics

- `/gnss/fix` (NavSatFix)

- `/imu/data` (Imu)

- `/scan_verified` (LaserScan), the strict stream

- `/scan_filtered` (LaserScan), the loose stream, with speckle

- `/sim/obstacle_markers` (MarkerArray), latched, ground truth polygons