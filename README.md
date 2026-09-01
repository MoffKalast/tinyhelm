![banner](tinyhelm/figs/banner.png)

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## About

![Status](https://img.shields.io/badge/status-work%20in%20progress-yellow)

A research-oriented guidance suite for marine vessels using ROS, loosely inspired by parts of MOOS-IvP and Nav2. The suite handles local planning, behaviours, and surface obstacle avoidance. Bring your own localization and thruster allocation, though most marine robots already have that part well sorted.

The suite is natively integrated with [Vizanti](https://github.com/MoffKalast/vizanti) as the ground control station, for global planning, operator teleop override, parameter setup and arming, etc.

To keep things simple there are two main modular abstractions:
- controllers, which take waypoints and publish cmd_vel
- monitors, which observe sensor data and suggest reactions

There is also a rudimentary headless 2D sim designed for ease of testing and tuning.

Live demo:
[![Obstacle avoidance testing](https://img.youtube.com/vi/gduNxYbaA6c/0.jpg)](https://www.youtube.com/watch?v=gduNxYbaA6c)

## Features

| Package |  |
| --- | --- |
| [`tinyhelm_core`](tinyhelm_core/ReadMe.md) | Behaviour arbitration, controller and monitor management, cmd_vel mux |
| [`tinyhelm_waypoints`](tinyhelm_waypoints/ReadMe.md) | Line-following controller for waypoint missions |
| [`tinyhelm_stationkeeping`](tinyhelm_stationkeeping/ReadMe.md) | Position hold controller |
| [`tinyhelm_obstacles`](tinyhelm_obstacles/ReadMe.md) | Costmap, obstacle monitor and Theta* path planner |
| [`tinyhelm_sim`](tinyhelm_sim/ReadMe.md) | Vessel dynamics and lidar simulator for testing without hardware |

## Installation

```bash
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/MoffKalast/tinyhelm.git

# Optional, visualization and control
git clone -b ros1 https://github.com/MoffKalast/vizanti.git 

cd ..
rosdep install -i --from-path src -y
catkin_make
```

Generally tested on 22.04 and 24.04 with ROS One. On 20.04 some apt packages may not be available, in that case install them with pip.