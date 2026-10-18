# Livox Avia Raspberry Pi Logger

ROS 2 based field logger for Livox Avia running on Raspberry Pi.

This is the software repository for the Raspberry Pi logger and the planned web interface. The current v1.0 provides command-line recording and monitoring; the web interface and API are planned additions in this same repository.

## Companion CAD project

The [Avia 35 mm pole mount CAD repository](https://github.com/TBRTK/avia-pole-mount-public) contains the FreeCAD models, STEP/STL print files, mounting instructions, assembly animation and complete-setup illustration. The [first mount prototype release](https://github.com/TBRTK/avia-pole-mount-public/releases/tag/v1.2.0-prototype.1) provides a complete downloadable ZIP, including the Reach RS4 Pro reference assembly.

## Logger v1.0

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

## Licensing scope

Original source code authored by Tore Berthelsen (TBRTK) is licensed under GPL-3.0-only. See [LICENSE](LICENSE) and [LICENSING_SCOPE.md](LICENSING_SCOPE.md). Third-party code, manufacturer content, images and CAD retain their existing rights; no new licence over that material is granted.

The software is provided AS IS, without warranty. Warranty exclusions and limitations of liability apply only to the extent permitted by GPLv3 sections 15–17 and applicable law.

## Support

If this project has been useful to you and you would like to support further testing, documentation, and development, you can buy me a coffee:

[☕ Buy me a coffee](https://buymeacoffee.com/tb_rtk)

Support is completely optional and is greatly appreciated.

## License

See [LICENSE](LICENSE).
