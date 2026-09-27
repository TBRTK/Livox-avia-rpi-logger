#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <utility>

using namespace std::chrono_literals;

class LivoxMonitor : public rclcpp::Node
{
public:
  LivoxMonitor()
  : Node("livox_monitor"),
    start_time_(Clock::now())
  {
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(250))
                     .reliable()
                     .durability_volatile();

    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                       .reliable()
                       .durability_volatile();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", imu_qos,
      std::bind(&LivoxMonitor::imuCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<livox_interfaces::msg::CustomMsg>(
      "/livox/lidar", lidar_qos,
      std::bind(&LivoxMonitor::lidarCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(1s, std::bind(&LivoxMonitor::display, this));

    RCLCPP_INFO(get_logger(), "Livox monitor started");
  }

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  static constexpr double RAD_TO_DEG = 57.29577951308232;
  static constexpr double PI = 3.14159265358979323846;

  static constexpr double JERK_WARN = 75.0;
  static constexpr double JERK_DEGRADED = 250.0;
  static constexpr double ACC_WARN = 0.35;
  static constexpr double ACC_DEGRADED = 0.80;
  static constexpr double ROT_WARN = 300.0;
  static constexpr double ROT_DEGRADED = 400.0;

  static constexpr double LIDAR_WARN_RATE = 9.5;
  static constexpr double LIDAR_DEG_RATE = 8.5;
  static constexpr double IMU_WARN_RATE = 190.0;
  static constexpr double IMU_DEG_RATE = 170.0;

  static constexpr double LIDAR_WARN_GAP = 0.20;
  static constexpr double LIDAR_DEG_GAP = 0.50;
  static constexpr double IMU_WARN_GAP = 0.030;
  static constexpr double IMU_DEG_GAP = 0.100;
  static constexpr double IMU_STREAM_WARN_GAP = 0.25;
  static constexpr double IMU_STREAM_DEG_GAP = 0.50;

  // High-frequency vibration monitor.
  // Thresholds are provisional until calibrated on this rig.
  static constexpr double VIB_CUTOFF_HZ = 20.0;
  static constexpr double VIB_WINDOW_SEC = 0.50;
  static constexpr double VIB_WARN_RMS = 0.060;
  static constexpr double VIB_DEG_RMS = 0.120;

  static constexpr const char *HEALTH_FILE =
    "/dev/shm/livox_avia_health";

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  TimePoint start_time_;
  std::deque<TimePoint> imu_times_;
  std::deque<TimePoint> lidar_times_;

  TimePoint last_imu_{};
  TimePoint last_lidar_{};
  TimePoint last_shock_{};

  bool have_imu_ = false;
  bool have_lidar_ = false;
  bool have_shock_ = false;

  double previous_stamp_ = 0.0;
  bool have_previous_imu_ = false;
  double previous_ax_ = 0.0;
  double previous_ay_ = 0.0;
  double previous_az_ = 0.0;

  double rotation_deg_s_ = 0.0;
  double accel_g_ = 1.0;
  double max_jerk_recent_ = 0.0;
  double max_accdev_recent_ = 0.0;
  double max_imu_sensor_gap_recent_ = 0.0;

  std::deque<TimePoint> rotation_warn_times_;
  std::deque<TimePoint> rotation_degraded_times_;

  bool vib_filter_initialized_ = false;
  double lp_ax_ = 0.0;
  double lp_ay_ = 0.0;
  double lp_az_ = 0.0;
  std::deque<std::pair<TimePoint, double>> vib_sq_samples_;
  double vibration_rms_ = 0.0;

  uint32_t point_num_ = 0;

  bool health_available_ = false;
  bool health_stale_ = false;
  int health_temp_ = 0;
  int health_volt_ = 0;
  int health_motor_ = 0;
  int health_dirty_ = 0;
  int health_firmware_ = 0;
  int health_device_ = 0;
  int health_fan_ = 0;
  int health_self_heating_ = 0;
  int health_system_ = 0;

  static double secondsBetween(const TimePoint &a, const TimePoint &b)
  {
    return std::chrono::duration<double>(a - b).count();
  }

  static void prune(std::deque<TimePoint> &q, const TimePoint &now, double seconds)
  {
    while (!q.empty() && secondsBetween(now, q.front()) > seconds) {
      q.pop_front();
    }
  }

  static double rate(std::deque<TimePoint> &q, const TimePoint &now)
  {
    prune(q, now, 2.0);

    if (q.size() < 2) return 0.0;

    const double dt = secondsBetween(q.back(), q.front());
    if (dt <= 0.0) return 0.0;

    return static_cast<double>(q.size() - 1) / dt;
  }

  static int statusLevel(const std::string &s)
  {
    if (s == "DEGRADED") return 2;
    if (s == "WARNING") return 1;
    return 0;
  }

  static std::string worstStatus(std::initializer_list<std::string> states)
  {
    std::string result = "OK";

    for (const auto &s : states) {
      if (statusLevel(s) > statusLevel(result)) {
        result = s;
      }
    }

    return result;
  }

  static std::string scalarHealthStatus(int value, bool severe_at_two = true)
  {
    if (value == 0) return "OK";
    if (severe_at_two && value >= 2) return "DEGRADED";
    return "WARNING";
  }

  void lidarCallback(const livox_interfaces::msg::CustomMsg::SharedPtr msg)
  {
    const auto now = Clock::now();
    have_lidar_ = true;
    last_lidar_ = now;
    lidar_times_.push_back(now);
    point_num_ = msg->point_num;
  }

  void updateVibration(
    const TimePoint &now,
    double dt,
    double ax,
    double ay,
    double az)
  {
    if (!vib_filter_initialized_) {
      lp_ax_ = ax;
      lp_ay_ = ay;
      lp_az_ = az;
      vib_filter_initialized_ = true;
      return;
    }

    if (dt <= 0.0 || dt > 0.05) return;

    const double rc = 1.0 / (2.0 * PI * VIB_CUTOFF_HZ);
    const double alpha = dt / (rc + dt);

    lp_ax_ += alpha * (ax - lp_ax_);
    lp_ay_ += alpha * (ay - lp_ay_);
    lp_az_ += alpha * (az - lp_az_);

    const double hx = ax - lp_ax_;
    const double hy = ay - lp_ay_;
    const double hz = az - lp_az_;

    const double vib_sq = hx * hx + hy * hy + hz * hz;

    vib_sq_samples_.push_back({now, vib_sq});

    while (
      !vib_sq_samples_.empty() &&
      secondsBetween(now, vib_sq_samples_.front().first) > VIB_WINDOW_SEC)
    {
      vib_sq_samples_.pop_front();
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto now = Clock::now();

    have_imu_ = true;
    last_imu_ = now;
    imu_times_.push_back(now);

    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    const double gyro_rad = std::sqrt(gx * gx + gy * gy + gz * gz);
    rotation_deg_s_ = gyro_rad * RAD_TO_DEG;

    accel_g_ = std::sqrt(ax * ax + ay * ay + az * az);

    if (rotation_deg_s_ >= ROT_WARN) {
      rotation_warn_times_.push_back(now);
    }

    if (rotation_deg_s_ >= ROT_DEGRADED) {
      rotation_degraded_times_.push_back(now);
    }

    prune(rotation_warn_times_, now, 1.0);
    prune(rotation_degraded_times_, now, 1.0);

    const double stamp =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    if (have_previous_imu_) {
      const double dt = stamp - previous_stamp_;

      if (dt > 0.0) {
        max_imu_sensor_gap_recent_ =
          std::max(max_imu_sensor_gap_recent_, dt);
      }

      updateVibration(now, dt, ax, ay, az);

      if (dt > 0.0 && dt < 0.1) {
        const double dax = ax - previous_ax_;
        const double day = ay - previous_ay_;
        const double daz = az - previous_az_;

        const double jerk =
          std::sqrt(dax * dax + day * day + daz * daz) / dt;

        const double accdev = std::fabs(accel_g_ - 1.0);

        max_jerk_recent_ = std::max(max_jerk_recent_, jerk);
        max_accdev_recent_ = std::max(max_accdev_recent_, accdev);

        if (jerk >= JERK_WARN || accdev >= ACC_WARN) {
          last_shock_ = now;
          have_shock_ = true;
        }
      }
    }

    previous_stamp_ = stamp;
    previous_ax_ = ax;
    previous_ay_ = ay;
    previous_az_ = az;
    have_previous_imu_ = true;
  }

  void calculateVibration(const TimePoint &now)
  {
    while (
      !vib_sq_samples_.empty() &&
      secondsBetween(now, vib_sq_samples_.front().first) > VIB_WINDOW_SEC)
    {
      vib_sq_samples_.pop_front();
    }

    if (vib_sq_samples_.size() < 30) {
      vibration_rms_ = 0.0;
      return;
    }

    double sum = 0.0;
    for (const auto &v : vib_sq_samples_) {
      sum += v.second;
    }

    vibration_rms_ =
      std::sqrt(sum / static_cast<double>(vib_sq_samples_.size()));
  }

  void readHealth()
  {
    health_available_ = false;
    health_stale_ = false;

    struct stat st {};
    if (stat(HEALTH_FILE, &st) != 0) {
      return;
    }

    std::ifstream file(HEALTH_FILE);
    if (!file.is_open()) return;

    std::string line;

    while (std::getline(file, line)) {
      const auto pos = line.find('=');
      if (pos == std::string::npos) continue;

      const std::string key = line.substr(0, pos);
      const std::string value = line.substr(pos + 1);

      int v = 0;
      try {
        v = std::stoi(value);
      } catch (...) {
        continue;
      }

      if (key == "temp") health_temp_ = v;
      else if (key == "volt") health_volt_ = v;
      else if (key == "motor") health_motor_ = v;
      else if (key == "dirty") health_dirty_ = v;
      else if (key == "firmware") health_firmware_ = v;
      else if (key == "device") health_device_ = v;
      else if (key == "fan") health_fan_ = v;
      else if (key == "self_heating") health_self_heating_ = v;
      else if (key == "system") health_system_ = v;
    }

    health_available_ = true;
  }

  void display()
  {
    const auto now = Clock::now();

    const double uptime = secondsBetween(now, start_time_);
    const double lidar_rate = rate(lidar_times_, now);
    const double imu_rate = rate(imu_times_, now);

    calculateVibration(now);
    readHealth();

    std::string lidar_status = "OK";
    std::string imu_status = "OK";
    std::string motion_status = "OK";
    std::string shock_status = "OK";
    std::string vibration_status = "OK";
    std::string health_status = "WAITING";
    std::string shock_trigger = "NONE";

    if (uptime > 3.0) {
      if (!have_lidar_) {
        lidar_status = "DEGRADED";
      } else {
        const double gap = secondsBetween(now, last_lidar_);

        if (gap >= LIDAR_DEG_GAP || lidar_rate < LIDAR_DEG_RATE) {
          lidar_status = "DEGRADED";
        } else if (gap >= LIDAR_WARN_GAP || lidar_rate < LIDAR_WARN_RATE) {
          lidar_status = "WARNING";
        }
      }

      if (!have_imu_) {
        imu_status = "DEGRADED";
      } else {
        const double stream_gap = secondsBetween(now, last_imu_);

        if (
          stream_gap >= IMU_STREAM_DEG_GAP ||
          max_imu_sensor_gap_recent_ >= IMU_DEG_GAP ||
          imu_rate < IMU_DEG_RATE)
        {
          imu_status = "DEGRADED";
        }
        else if (
          stream_gap >= IMU_STREAM_WARN_GAP ||
          max_imu_sensor_gap_recent_ >= IMU_WARN_GAP ||
          imu_rate < IMU_WARN_RATE)
        {
          imu_status = "WARNING";
        }
      }
    }

    prune(rotation_warn_times_, now, 1.0);
    prune(rotation_degraded_times_, now, 1.0);

    const double warn_duration =
      rotation_warn_times_.size() / std::max(imu_rate, 1.0);

    const double degraded_duration =
      rotation_degraded_times_.size() / std::max(imu_rate, 1.0);

    if (degraded_duration >= 0.20) {
      motion_status = "DEGRADED";
    } else if (warn_duration >= 0.20) {
      motion_status = "WARNING";
    }

    if (have_shock_ && secondsBetween(now, last_shock_) < 2.0) {
      const bool jerk_trigger = max_jerk_recent_ >= JERK_WARN;
      const bool accel_trigger = max_accdev_recent_ >= ACC_WARN;

      if (jerk_trigger && accel_trigger) {
        shock_trigger = "JERK+ACCEL";
      } else if (jerk_trigger) {
        shock_trigger = "JERK";
      } else if (accel_trigger) {
        shock_trigger = "ACCEL";
      }

      if (
        max_jerk_recent_ >= JERK_DEGRADED ||
        max_accdev_recent_ >= ACC_DEGRADED)
      {
        shock_status = "DEGRADED";
      }
      else if (jerk_trigger || accel_trigger)
      {
        shock_status = "WARNING";
      }
    }
    else
    {
      max_jerk_recent_ = 0.0;
      max_accdev_recent_ = 0.0;
      have_shock_ = false;
    }

    if (vibration_rms_ >= VIB_DEG_RMS) {
      vibration_status = "DEGRADED";
    } else if (vibration_rms_ >= VIB_WARN_RMS) {
      vibration_status = "WARNING";
    }

    std::string temp_status = "WAITING";
    std::string volt_status = "WAITING";
    std::string motor_status = "WAITING";
    std::string optics_status = "WAITING";
    std::string firmware_status = "WAITING";
    std::string device_status = "WAITING";
    std::string fan_status = "WAITING";
    std::string system_status = "WAITING";

    if (health_available_) {
      temp_status = scalarHealthStatus(health_temp_);
      volt_status = scalarHealthStatus(health_volt_);
      motor_status = scalarHealthStatus(health_motor_);
      optics_status = scalarHealthStatus(health_dirty_, false);
      firmware_status =
        (health_firmware_ == 0 ? "OK" : "DEGRADED");
      device_status = scalarHealthStatus(health_device_, false);
      fan_status = scalarHealthStatus(health_fan_, false);
      system_status = scalarHealthStatus(health_system_);

      health_status = worstStatus({
        temp_status,
        volt_status,
        motor_status,
        optics_status,
        firmware_status,
        device_status,
        fan_status,
        system_status
      });

    }

    const std::string overall =
      worstStatus({
        lidar_status,
        imu_status,
        motion_status,
        shock_status,
        vibration_status,
        health_status
      });

    std::cout << "\033[2J\033[H";

    std::cout
      << "========================================\n"
      << "       LIVOX AVIA LIVE MONITOR v1.0\n"
      << "========================================\n\n"
      << "QUALITY      : " << overall << "\n\n";

    std::cout
      << "LIDAR        : "
      << std::setw(9) << std::left << lidar_status
      << std::right << std::fixed << std::setprecision(2)
      << std::setw(7) << lidar_rate << " Hz"
      << "   points " << point_num_ << "\n";

    std::cout
      << "IMU          : "
      << std::setw(9) << std::left << imu_status
      << std::right << std::fixed << std::setprecision(1)
      << std::setw(7) << imu_rate << " Hz\n";

    std::cout
      << "IMU GAP      : "
      << std::fixed << std::setprecision(1)
      << max_imu_sensor_gap_recent_ * 1000.0
      << " ms (Avia timestamp)\n\n";

    std::cout
      << "MOTION       : "
      << std::setw(9) << std::left << motion_status
      << std::right << std::fixed << std::setprecision(1)
      << std::setw(8) << rotation_deg_s_ << " deg/s\n";

    std::cout
      << "SHOCK        : "
      << std::setw(9) << std::left << shock_status
      << std::right << std::fixed << std::setprecision(1)
      << " jerk " << std::setw(7) << max_jerk_recent_ << " g/s\n";

    std::cout
      << "ACCEL DEV    : "
      << std::fixed << std::setprecision(3)
      << max_accdev_recent_ << " g"
      << "   trigger " << shock_trigger << "\n";

    std::cout
      << "VIBRATION    : "
      << std::setw(9) << std::left << vibration_status
      << std::right << " RMS "
      << std::fixed << std::setprecision(4)
      << vibration_rms_ << " g  (>20 Hz, 0.5 s)\n";

    std::cout
      << "ACCEL        : "
      << std::fixed << std::setprecision(3)
      << accel_g_ << " g\n\n";

    std::cout
      << "AVIA HEALTH  : " << health_status;

    std::cout << "\n";

    std::cout
      << "  TEMP       : " << temp_status
      << " (" << health_temp_ << ")\n"
      << "  VOLTAGE    : " << volt_status
      << " (" << health_volt_ << ")\n"
      << "  MOTOR      : " << motor_status
      << " (" << health_motor_ << ")\n"
      << "  OPTICS     : " << optics_status
      << " (" << health_dirty_ << ")\n"
      << "  FIRMWARE   : " << firmware_status
      << " (" << health_firmware_ << ")\n"
      << "  DEVICE     : " << device_status
      << " (" << health_device_ << ")\n"
      << "  FAN        : " << fan_status
      << " (" << health_fan_ << ")\n"
      << "  SYSTEM     : " << system_status
      << " (" << health_system_ << ")\n"
      << "  HEATING    : "
      << (health_self_heating_ ? "ACTIVE" : "OFF")
      << " (" << health_self_heating_ << ")\n\n";

    std::cout

      << "Ctrl+C avslutter monitoren.\n"
      << "========================================\n"
      << std::flush;

    max_imu_sensor_gap_recent_ = 0.0;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
