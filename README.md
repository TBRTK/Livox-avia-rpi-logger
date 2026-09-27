# Livox Avia Raspberry Pi Logger

ROS 2 based field logger for Livox Avia running on Raspberry Pi.

Current v1.0 features:

- Livox Avia ROS 2 driver fixes
- `/livox/lidar` CustomMsg recording
- `/livox/imu` recording
- Avia health monitoring
- motion, shock and vibration monitoring
- automatic rosbag recording
- disk-space protection
- clean Ctrl+C shutdown
- field monitor logging and warning summary

## Platform

- Raspberry Pi 3 Model B+
- Ubuntu Server 22.04
- ROS 2 Humble
- Livox Avia
- Livox SDK 2.3.x

## Repository structure

- `livox_monitor/` - ROS 2 monitoring package
- `scripts/` - field recording scripts
- `patches/` - required Livox ROS 2 driver modifications

## Status

Logger and monitor v1.0 are tested and ready for field use.

Next milestone: offline FAST-LIO processing of recorded Avia LiDAR + IMU data.
