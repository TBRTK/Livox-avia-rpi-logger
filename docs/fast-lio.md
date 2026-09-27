# FAST-LIO offline processing

This document describes the tested offline FAST-LIO workflow for rosbag data recorded by this Livox Avia Raspberry Pi logger.

The current setup keeps FAST-LIO off the Raspberry Pi. The Pi records Livox Avia LiDAR and internal IMU data, and a stronger x86_64 computer processes the bag offline.

## Tested environment

- Ubuntu 22.04.5 LTS under WSL2
- ROS 2 Humble Desktop
- WSLg / RViz2
- `Ericsii/FAST_LIO_ROS2`, branch `ros2`
- Upstream FAST-LIO commit: `2fffc57`
- Livox message package: `livox_interfaces` from the Raspberry Pi logger workspace

The tested rosbag contained:

- `/livox/lidar` — `livox_interfaces/msg/CustomMsg`
- `/livox/imu` — `sensor_msgs/msg/Imu`
- LiDAR: approximately 10 Hz
- IMU: approximately 202 Hz

## Why a patch is required

`Ericsii/FAST_LIO_ROS2` normally depends on `livox_ros_driver2` and uses:

```text
livox_ros_driver2/msg/CustomMsg
```

This logger uses the older ROS 2 Livox message package:

```text
livox_interfaces/msg/CustomMsg
```

The relevant `CustomMsg` / `CustomPoint` fields match the FAST-LIO Avia preprocessing path, including per-point `offset_time`.

The patch in this repository changes FAST-LIO from `livox_ros_driver2` to `livox_interfaces` and enables map accumulation required by the `/map_save` service:

```text
patches/fast_lio_ros2_livox_interfaces.patch
```

The patch was verified with `git apply --check` against upstream commit `2fffc57`.

## 1. Install ROS 2 Humble

Use Ubuntu 22.04 and install ROS 2 Humble Desktop. RViz2 was tested successfully under WSLg.

After installation:

```bash
source /opt/ros/humble/setup.bash
```

For automatic sourcing:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

## 2. Create the FAST-LIO workspace

Keep the workspace in the Linux filesystem rather than under `/mnt/c` when using WSL2.

```bash
mkdir -p ~/fastlio_ws/src
cd ~/fastlio_ws/src
```

Clone the tested FAST-LIO ROS 2 fork:

```bash
git clone --branch ros2 --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
cd FAST_LIO_ROS2
git checkout 2fffc57
```

## 3. Add `livox_interfaces`

Copy the same `livox_interfaces` package used by the logger into the FAST-LIO workspace.

Example when the Raspberry Pi logger is reachable at `192.168.0.74`:

```bash
cd ~/fastlio_ws/src
scp -r tore@192.168.0.74:~/ws_livox/src/livox_ros2_driver/livox_interfaces .
```

The message definition used by the tested logger contains the FAST-LIO Avia fields:

```text
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
```

with each point containing:

```text
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
```

## 4. Apply the FAST-LIO patch

Copy `patches/fast_lio_ros2_livox_interfaces.patch` from this repository to the processing computer, then apply it from the FAST-LIO source directory:

```bash
cd ~/fastlio_ws/src/FAST_LIO_ROS2
git apply /path/to/fast_lio_ros2_livox_interfaces.patch
```

Optional verification before applying:

```bash
git apply --check /path/to/fast_lio_ros2_livox_interfaces.patch
```

The patch changes:

- `CMakeLists.txt`
- `package.xml`
- `src/preprocess.h`
- `src/preprocess.cpp`
- `src/laserMapping.cpp`
- `config/avia.yaml`

The code dependency changes from `livox_ros_driver2` to `livox_interfaces`.

The Avia configuration also enables:

```yaml
publish:
    map_en: true
```

This is required by the current ROS 2 fork because `/map_save` writes the accumulated `pcl_wait_pub` cloud, which is populated by `publish_map()` only when `publish.map_en` is enabled.

## 5. Install dependencies and build

From the workspace root:

```bash
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build in Release mode:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then source the workspace:

```bash
source ~/fastlio_ws/install/setup.bash
```

A successful build should expose:

```bash
ros2 pkg executables fast_lio
```

Expected executable:

```text
fast_lio fastlio_mapping
```

## 6. Avia configuration

The tested FAST-LIO Avia configuration uses:

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 1
    scan_line: 6
```

The upstream Avia extrinsic values were left unchanged for the initial test:

```yaml
mapping:
    extrinsic_est_en: false
    extrinsic_T: [0.04165, 0.02326, -0.0284]
    extrinsic_R: [1., 0., 0.,
                  0., 1., 0.,
                  0., 0., 1.]
```

These values should not be assumed to be a final calibrated extrinsic solution for another hardware arrangement.

## 7. Timestamp verification

The logger was checked before the first FAST-LIO run.

For a LiDAR message:

```text
header.stamp = 71 s + 699767350 ns
             = 71699767350 ns

timebase     = 71699767350 ns
```

So the recorded `CustomMsg` had:

```text
header.stamp == timebase
```

The nearest IMU samples for overlapping LiDAR frames were approximately 0.6–1.6 ms away in the test capture, consistent with the approximately 202 Hz Avia IMU stream.

No additional LiDAR-to-IMU time offset was used in the initial FAST-LIO test.

## 8. IMU acceleration units

The Livox logger publishes the raw Avia acceleration values, which are approximately `1.0` at rest rather than ROS-standard `9.81 m/s²`.

The tested FAST-LIO code normalizes the measured mean acceleration during IMU initialization and scales it using `G_m_s2`:

```cpp
acc_avr = acc_avr * G_m_s2 / mean_acc.norm();
```

Therefore no separate bag conversion from `g` to `m/s²` was required for this tested pipeline.

## 9. Run FAST-LIO

Start FAST-LIO with the Avia configuration:

```bash
cd ~/fastlio_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml rviz:=true
```

Expected startup includes:

```text
p_pre->lidar_type 1
Multi thread started
Node init finished.
```

When bag data starts, the tested run reported:

```text
IMU Initial Done
Initialize the map kdtree
```

A single `No point, skip this scan!` message was observed during startup and did not prevent mapping.

## 10. Play a recorded bag

In a second terminal:

```bash
cd ~/fastlio_ws
source install/setup.bash
ros2 bag play ~/bags/avia_test_60s --rate 0.5
```

The first test used half speed for easier observation. The approximately 54.65 second bag therefore took about 109 seconds to play.

RViz should show the accumulated FAST-LIO point cloud while the bag is playing.

## 11. Save the map

The ROS 2 fork exposes:

```text
/map_save
```

Do not stop FAST-LIO before calling the service.

The default `avia.yaml` path is:

```yaml
map_file_path: "./test.pcd"
```

For a predictable output location, set an absolute path before starting FAST-LIO, for example:

```yaml
map_file_path: "/home/user/fastlio_ws/maps/avia_test_60s.pcd"
```

Create the destination directory if needed:

```bash
mkdir -p ~/fastlio_ws/maps
```

After bag playback finishes, save while FAST-LIO is still running:

```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

Expected response:

```text
success=True, message='Map saved.'
```

The tested 54.65 second bench bag produced a binary PCD containing:

```text
POINTS 59956
DATA binary
```

The resulting file was approximately 1.9 MB.

## 12. Inspect the PCD

Install PCL tools if required:

```bash
sudo apt install -y pcl-tools
```

Open the saved map:

```bash
pcl_viewer ~/fastlio_ws/maps/avia_test_60s.pcd
```

The saved test map was successfully opened and visually inspected with `pcl_viewer`.

## Current status

The following pipeline has been tested successfully:

```text
Livox Avia
  -> Raspberry Pi ROS 2 logger
  -> rosbag2
  -> livox_interfaces/CustomMsg + sensor_msgs/Imu
  -> FAST-LIO ROS 2
  -> RViz
  -> /map_save
  -> binary PCD
```

The next stage is testing longer mobile field recordings and evaluating mapping quality before integrating the OpenRTK330LI RTK/INS solution.
