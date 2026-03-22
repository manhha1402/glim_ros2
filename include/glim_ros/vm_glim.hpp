#pragma once

#include <any>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/vm_glim_parameters.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <std_srvs/srv/trigger.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif
#include <open3d/Open3D.h>
namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class VMGlim : public rclcpp::Node {
public:
  VMGlim(const rclcpp::NodeOptions& options);
  ~VMGlim();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  size_t livox_custom_callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  double min_lidar_frame_interval;
  int min_points_to_process;
  double last_processed_points_stamp;
  int processed_frame_count;
  bool dump_on_unload;

  // Scan recording (LC-corrected submaps)
  bool save_scans_;
  std::string save_scans_path_;
  std::set<int> saved_submap_ids_;
  std::mutex save_mutex_;

  std::shared_ptr<vm_glim::ParamListener> param_listener_;
  vm_glim::Params params_;
  rclcpp::TimerBase::SharedPtr timer_;
  void params_callback();
  std::string intensity_field, ring_field;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Time last_map_pub_time_;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
