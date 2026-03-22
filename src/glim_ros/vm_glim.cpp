#include <glim_ros/vm_glim.hpp>
#include <glim_ros/vm_glim_parameters.hpp>

#define GLIM_ROS2

#include <deque>
#include <fstream>
#include <iomanip>
#include <thread>
#include <iostream>
#include <functional>
#include <filesystem>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>

#include <open3d/Open3D.h>

namespace glim {

VMGlim::VMGlim(const rclcpp::NodeOptions& options) : Node("vm_glim", options) {
  param_listener_ = std::make_shared<vm_glim::ParamListener>(this);
  params_ = param_listener_->get_params();

  dump_on_unload = params_.dump_on_unload;

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  save_scans_ = params_.save_scans;
  save_scans_path_ = params_.save_scans_path;

  if (save_scans_) {
    std::filesystem::create_directories(save_scans_path_);
    spdlog::info("save_scans enabled: writing LC-corrected submaps to {}", save_scans_path_);

    // Register callback: fires after every global optimization (including loop closures).
    // submap->T_world_origin is the LC-corrected pose; submap->frame holds the merged
    // point cloud in the submap-origin frame (never dropped by global mapping).
    GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) {
      std::lock_guard<std::mutex> lock(save_mutex_);

      for (const auto& submap : submaps) {
        if (!submap->frame || submap->frame->num_points == 0) {
          continue;
        }

        // Save PLY once per submap (points are in submap-origin frame, unchanged by LC)
        if (!saved_submap_ids_.count(submap->id)) {
          open3d::geometry::PointCloud o3d_pcd;
          o3d_pcd.points_.resize(submap->frame->num_points);
          for (size_t i = 0; i < submap->frame->num_points; ++i) {
            o3d_pcd.points_[i] = submap->frame->points[i].head<3>();
          }
          if (submap->frame->intensities) {
            o3d_pcd.colors_.resize(submap->frame->num_points);
            for (size_t i = 0; i < submap->frame->num_points; ++i) {
              const double v = std::max(0.0, std::min(1.0, submap->frame->intensities[i] / 255.0));
              o3d_pcd.colors_[i] = Eigen::Vector3d(v, v, v);
            }
          }
          const std::string ply_path = save_scans_path_ + "/submap_" + std::to_string(submap->id) + ".ply";
          if (open3d::io::WritePointCloud(ply_path, o3d_pcd)) {
            saved_submap_ids_.insert(submap->id);
          } else {
            spdlog::warn("save_scans: failed to write {}", ply_path);
          }
        }
      }

      // Overwrite poses.txt with the latest LC-corrected T_world_origin for every saved submap.
      // Reconstruction: p_world = T_world_origin * p_submap_origin (from PLY)
      std::ofstream poses_file(save_scans_path_ + "/poses.txt", std::ios::trunc);
      poses_file << "# submap_id  tx ty tz  qx qy qz qw  (T_world_origin, LC-corrected)\n";
      for (const auto& submap : submaps) {
        if (!saved_submap_ids_.count(submap->id)) {
          continue;
        }
        const Eigen::Quaterniond q(submap->T_world_origin.rotation());
        const Eigen::Vector3d t(submap->T_world_origin.translation());
        poses_file << submap->id << " " << std::fixed << std::setprecision(9) << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                   << q.w() << "\n";
      }
      spdlog::info("save_scans: updated poses.txt ({} submaps)", saved_submap_ids_.size());
    });
  }

  std::string config_path = params_.config_path;

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  keep_raw_points = params_.keep_raw_points;
  imu_time_offset = params_.imu_time_offset;
  points_time_offset = params_.points_time_offset;
  acc_scale = params_.acc_scale;
  min_lidar_frame_interval = params_.min_lidar_frame_interval;
  min_points_to_process = params_.min_points_to_process;
  last_processed_points_stamp = -1.0;
  processed_frame_count = 0;

  // glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = params_.intensity_field;  // config_sensors.param<std::string>("sensors", "intensity_field", "intensity");
  ring_field = params_.ring_field;            // config_sensors.param<std::string>("sensors", "ring_field", "");

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  // if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
  if (params_.enable_local_mapping) {
    const std::string sub_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
    if (!sub_mapping_so_name.empty()) {
      spdlog::info("load {}", sub_mapping_so_name);
      auto sub = SubMappingBase::load_module(sub_mapping_so_name);
      if (sub) {
        sub_mapping.reset(new AsyncSubMapping(sub));
      }
    }
  }

  // Global mapping
  // if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
  if (params_.enable_global_mapping) {
    const std::string global_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
    if (!global_mapping_so_name.empty()) {
      spdlog::info("load {}", global_mapping_so_name);
      auto global = GlobalMappingBase::load_module(global_mapping_so_name);
      if (global) {
        global_mapping.reset(new AsyncGlobalMapping(global));
      }
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError& e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;
  const std::string imu_topic = params_.imu_topic;        // config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic = params_.points_topic;  // config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic = params_.image_topic;    // config_ros.param<std::string>("glim_ros", "image_topic", "");

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&VMGlim::imu_callback, this, _1));

  qos = get_qos_settings(config_ros, "glim_ros", "points_qos");
  if (params_.points_msg_type == "LivoxCustomMsg") {
    spdlog::info("subscribing to Livox CustomMsg on {}", points_topic);
    livox_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(points_topic, qos, std::bind(&VMGlim::livox_custom_callback, this, _1));
  } else {
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, qos, std::bind(&VMGlim::points_callback, this, _1));
  }

  // PointCloud2 publisher (same data as glim viewer / rviz_viewer ~/points)
  // points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);

  // Global map publisher (merged submaps in map frame; throttled)
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();
  // map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map", map_qos);
  last_map_pub_time_ = this->now();

#ifdef BUILD_WITH_CV_BRIDGE
  qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
  image_sub = image_transport::create_subscription(this, image_topic, std::bind(&VMGlim::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

  spdlog::debug("initialized");
}

VMGlim::~VMGlim() {
  spdlog::debug("quit");
  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/dump";
    wait(true);
    save(dump_path);
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& VMGlim::extension_subscriptions() {
  return extension_subs;
}

void VMGlim::params_callback() {
  if (param_listener_->is_old(params_)) {
    spdlog::info("Parameters updated, reconfiguring subscribers...");
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  }
}

void VMGlim::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if (!time_keeper->validate_imu_stamp(imu_stamp)) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

#ifdef BUILD_WITH_CV_BRIDGE
void VMGlim::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}
#endif

size_t VMGlim::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }
  if (min_points_to_process > 0 && static_cast<int>(raw_points->size()) < min_points_to_process) {
    spdlog::debug("skip sparse PointCloud2 frame ({} < {} points)", raw_points->size(), min_points_to_process);
    return 0;
  }
  const double frame_stamp = raw_points->stamp;
  if (min_lidar_frame_interval > 0 && last_processed_points_stamp >= 0 && (frame_stamp - last_processed_points_stamp) < min_lidar_frame_interval) {
    spdlog::debug("skip rapid PointCloud2 frame (interval={:.4f}s < {:.4f}s)", frame_stamp - last_processed_points_stamp, min_lidar_frame_interval);
    return 0;
  }

  raw_points->stamp += points_time_offset;
  raw_points->frame_index = processed_frame_count;
  if (!time_keeper->process(raw_points)) {
    spdlog::warn("skip an invalid point cloud (stamp={}, frame_index={})", raw_points->stamp, raw_points->frame_index);
    return 0;
  }
  processed_frame_count++;
  auto preprocessed = preprocessor->preprocess(raw_points);
  if (preprocessed == nullptr) {
    spdlog::warn("skipping point cloud (preprocessing returned null, stamp={})", raw_points->stamp);
    return 0;
  }

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);
  last_processed_points_stamp = frame_stamp;

  const size_t workload = odometry_estimation->workload();
  spdlog::debug("workload={}", workload);

  return workload;
}

namespace {
RawPoints::Ptr extract_raw_points_from_livox(const livox_ros_driver2::msg::CustomMsg& msg) {
  auto raw = std::make_shared<RawPoints>();
  raw->stamp = static_cast<double>(msg.timebase) / 1e9;
  raw->frame_id = msg.header.frame_id;
  const size_t n = msg.points.size();
  raw->points.resize(n);
  raw->times.resize(n);
  raw->intensities.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto& p = msg.points[i];
    raw->points[i] << p.x, p.y, p.z, 1.0;
    raw->times[i] = static_cast<double>(p.offset_time) / 1e9;
    raw->intensities[i] = static_cast<double>(p.reflectivity);
  }
  return raw;
}
}  // namespace

size_t VMGlim::livox_custom_callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
  spdlog::trace("livox: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  auto raw_points = extract_raw_points_from_livox(*msg);
  if (raw_points->size() == 0) {
    spdlog::warn("Livox CustomMsg has no points");
    return 0;
  }
  if (min_points_to_process > 0 && static_cast<int>(raw_points->size()) < min_points_to_process) {
    spdlog::debug("skip sparse Livox frame ({} < {} points)", raw_points->size(), min_points_to_process);
    return 0;
  }
  const double frame_stamp = raw_points->stamp;
  if (min_lidar_frame_interval > 0 && last_processed_points_stamp >= 0 && (frame_stamp - last_processed_points_stamp) < min_lidar_frame_interval) {
    spdlog::debug("skip rapid Livox frame (interval={:.4f}s < {:.4f}s)", frame_stamp - last_processed_points_stamp, min_lidar_frame_interval);
    return 0;
  }

  raw_points->stamp += points_time_offset;
  raw_points->frame_index = processed_frame_count;
  if (!time_keeper->process(raw_points)) {
    spdlog::warn("skip an invalid point cloud (stamp={}, frame_index={})", raw_points->stamp, raw_points->frame_index);
    return 0;
  }
  processed_frame_count++;
  auto preprocessed = preprocessor->preprocess(raw_points);
  if (preprocessed == nullptr) {
    spdlog::warn("skipping point cloud (preprocessing returned null, stamp={})", raw_points->stamp);
    return 0;
  }

  if (keep_raw_points) {
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);
  last_processed_points_stamp = frame_stamp;
  const size_t workload = odometry_estimation->workload();
  spdlog::debug("workload={}", workload);
  return workload;
}

bool VMGlim::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void VMGlim::timer_callback() {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
    }
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  // Publish latest scan as PointCloud2 (similar to glim viewer / rviz_viewer)
  // if (points_pub_->get_subscription_count() > 0 && !estimation_frames.empty() && estimation_frames.back()->frame) {
  //   const auto& new_frame = estimation_frames.back();
  //   std::string frame_id_str;
  //   switch (new_frame->frame_id) {
  //     case glim::FrameID::LIDAR:
  //       frame_id_str = params_.lidar_frame_id;
  //       break;
  //     case glim::FrameID::IMU:
  //       frame_id_str = params_.imu_frame_id;
  //       break;
  //     case glim::FrameID::WORLD:
  //       frame_id_str = params_.map_frame_id;
  //       break;
  //   }
  //   auto points_msg = glim::frame_to_pointcloud2(frame_id_str, new_frame->stamp, *new_frame->frame);
  //   points_pub_->publish(*points_msg);
  // }

  // Publish global map (throttled; expensive)
  // if (global_mapping && map_pub_->get_subscription_count() > 0) {
  //   const rclcpp::Time now = this->now();
  //   if ((now - last_map_pub_time_).seconds() >= 10.0) {
  //     last_map_pub_time_ = now;
  //     auto global_points = global_mapping->export_points();
  //     if (global_points && global_points->size() > 0) {
  //       auto map_msg = glim::frame_to_pointcloud2(params_.map_frame_id, now.seconds(), *global_points);
  //       map_pub_->publish(*map_msg);
  //     }
  //   }
  // }

  if (sub_mapping) {
    for (const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

void VMGlim::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void VMGlim::save(const std::string& path) {
  if (global_mapping) {
    global_mapping->save(path);
    auto global_points = global_mapping->export_points();
    if (global_points && global_points->size() > 0) {
      open3d::geometry::PointCloud pcd;
      pcd.points_.resize(global_points->size());
      for (size_t i = 0; i < global_points->size(); ++i) {
        pcd.points_[i] = global_points->points[i].head<3>().cast<double>();
      }
      if (global_points->intensities) {
        pcd.colors_.resize(global_points->size());
        for (size_t i = 0; i < global_points->size(); ++i) {
          const double v = global_points->intensities[i];
          const double vn = std::max(0., std::min(1., v / 255.));
          pcd.colors_[i] = Eigen::Vector3d(vn, vn, vn);
        }
      }
      const std::string ply_path = path + "/global_map.ply";
      if (open3d::io::WritePointCloud(ply_path, pcd)) {
        spdlog::info("saved global map to {}", ply_path);
      } else {
        spdlog::warn("failed to write PLY to {}", ply_path);
      }
    }
  }
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::VMGlim);