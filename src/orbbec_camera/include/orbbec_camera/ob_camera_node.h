/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <image_publisher/image_publisher.hpp>
#include <image_transport/publisher.hpp>
#include "libobsensor/ObSensor.hpp"

#include "orbbec_camera_msgs/msg/device_info.hpp"
#include "orbbec_camera_msgs/srv/get_device_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include "orbbec_camera_msgs/srv/get_int32.hpp"
#include "orbbec_camera_msgs/srv/get_string.hpp"
#include "orbbec_camera_msgs/srv/set_int32.hpp"
#include "orbbec_camera_msgs/srv/get_bool.hpp"
#include "orbbec_camera_msgs/srv/set_string.hpp"
#include "orbbec_camera/constants.h"
#include "orbbec_camera/dynamic_params.h"
#include "orbbec_camera/d2c_viewer.h"
#include "magic_enum/magic_enum.hpp"

#define STREAM_NAME(sip)                                                                       \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                      \
                                     << _stream_name[sip.first]                                \
                                     << ((sip.second > 0) ? std::to_string(sip.second) : ""))) \
      .str()
#define FRAME_ID(sip)                                                                              \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                          \
                                     << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_frame")) \
      .str()
#define OPTICAL_FRAME_ID(sip)                                                                     \
  (static_cast<std::ostringstream&&>(                                                             \
       std::ostringstream() << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_optical_frame")) \
      .str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip)                                            \
  (static_cast<std::ostringstream&&>(std::ostringstream()                         \
                                     << getNamespaceStr() << "_aligned_depth_to_" \
                                     << STREAM_NAME(sip) << "_frame"))            \
      .str()
#define BASE_FRAME_ID() \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_link")).str()
#define ODOM_FRAME_ID()                                                                           \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_odom_frame")) \
      .str()

namespace orbbec_camera {
using GetDeviceInfo = orbbec_camera_msgs::srv::GetDeviceInfo;
using Extrinsics = orbbec_camera_msgs::msg::Extrinsics;
using SetInt32 = orbbec_camera_msgs::srv::SetInt32;
using GetInt32 = orbbec_camera_msgs::srv::GetInt32;
using GetString = orbbec_camera_msgs::srv::GetString;
using SetString = orbbec_camera_msgs::srv::SetString;
using SetBool = std_srvs::srv::SetBool;
using GetBool = orbbec_camera_msgs::srv::GetBool;

typedef std::pair<ob_stream_type, int> stream_index_pair;

const stream_index_pair COLOR{OB_STREAM_COLOR, 0};
const stream_index_pair DEPTH{OB_STREAM_DEPTH, 0};
const stream_index_pair INFRA0{OB_STREAM_IR, 0};
const stream_index_pair INFRA1{OB_STREAM_IR, 1};
const stream_index_pair INFRA2{OB_STREAM_IR, 2};

const stream_index_pair GYRO{OB_STREAM_GYRO, 0};
const stream_index_pair ACCEL{OB_STREAM_ACCEL, 0};

const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, COLOR};

const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL};

class OBCameraNode {
 public:
  OBCameraNode(rclcpp::Node* node, std::shared_ptr<ob::Device> device,
               std::shared_ptr<Parameters> parameters);

  template <class T>
  void setAndGetNodeParameter(
      T& param, const std::string& param_name, const T& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor());  // set and get parameter

  ~OBCameraNode();

  void clean();

 private:
  void setupDevices();

  void setupProfiles();

  void getParameters();

  void setupTopics();

  void setupPipelineConfig();

  void setupCameraCtrlServices();

  void startStreams();

  void stopStreams();

  void setupDefaultImageFormat();

  void setupPublishers();

  void publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                       const tf2::Quaternion& q, const std::string& from, const std::string& to);

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  std::optional<OBCameraParam> findDefaultCameraParam();

  std::optional<OBCameraParam> getDepthCameraParam();

  std::optional<OBCameraParam> getColorCameraParam();

  void getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                           std::shared_ptr<GetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                           std::shared_ptr<SetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                       std::shared_ptr<GetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void setGainCallback(const std::shared_ptr<SetInt32::Request>& request,
                       std::shared_ptr<SetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void getWhiteBalanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                               std::shared_ptr<GetInt32::Response>& response);

  void setWhiteBalanceCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                               std::shared_ptr<SetInt32 ::Response>& response);

  void getAutoWhiteBalanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response);

  void setAutoWhiteBalanceCallback(const std::shared_ptr<SetBool::Request>& request,
                                   std::shared_ptr<SetBool::Response>& response);

  void setAutoExposureCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
                               const stream_index_pair& stream_index);

  void setLaserEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFloorEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setLdpEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFanWorkModeCallback(const std::shared_ptr<SetInt32::Request>& request,
                              std::shared_ptr<SetInt32::Response>& response);

  void getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                             std::shared_ptr<GetDeviceInfo::Response>& response);

  void getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                     std::shared_ptr<GetString::Response>& response);

  void toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                            std::shared_ptr<SetBool::Response>& response,
                            const stream_index_pair& stream_index);

  void setMirrorCallback(const std::shared_ptr<SetBool::Request>& request,
                         std::shared_ptr<SetBool::Response>& response,
                         const stream_index_pair& stream_index);

  void getLdpStatusCallback(const std::shared_ptr<GetBool::Request>& request,
                            std::shared_ptr<GetBool::Response>& response);

  bool toggleSensor(const stream_index_pair& stream_index, bool enabled, std::string& msg);

  void saveImageCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
                         std::shared_ptr<std_srvs::srv::Empty::Response>& response);

  void savePointCloudCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
                              std::shared_ptr<std_srvs::srv::Empty::Response>& response);

  void switchIRCameraCallback(const std::shared_ptr<SetString::Request>& request,
                              std::shared_ptr<SetString::Response>& response);

  void publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  void publishDepthPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  void publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  void onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet>& frame_set);

  void onNewFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                          const stream_index_pair& stream_index);

  void saveImageToQueue(const stream_index_pair& stream_index, const cv::Mat& image,
                        const sensor_msgs::msg::Image::SharedPtr& image_msg,
                        uint64_t hardware_timestamp, uint64_t system_timestamp, uint64_t index);

  void saveImageThread();

  bool setupFormatConvertType(OBFormat format);

 private:
  rclcpp::Node* node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  rclcpp::Logger logger_;
  std::string camera_name_ = "camera";
  std::atomic_bool is_running_{false};
  std::unique_ptr<ob::Pipeline> pipeline_ = nullptr;
  std::atomic_bool pipeline_started_{false};
  std::shared_ptr<ob::Config> pipeline_config_ = nullptr;
  std::map<stream_index_pair, std::shared_ptr<ob::Sensor>> sensors_;
  std::map<stream_index_pair, ob_camera_intrinsic> stream_intrinsics_;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> camera_infos_;
  std::map<stream_index_pair, OBCameraParam> ob_camera_param_;
  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, int> fps_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::string camera_link_frame_id_;
  bool depth_registration_ = false;
  std::map<stream_index_pair, std::string> image_qos_;
  std::map<stream_index_pair, std::string> camera_info_qos_;
  std::map<stream_index_pair, ob_format> format_;
  std::map<stream_index_pair, std::string> format_str_;
  std::map<stream_index_pair, int> image_format_;
  std::map<stream_index_pair, std::vector<std::shared_ptr<ob::VideoStreamProfile>>>
      supported_profiles_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
  std::map<stream_index_pair, uint32_t> seq_;
  std::map<stream_index_pair, cv::Mat> images_;
  std::map<stream_index_pair, std::string> encoding_;
  std::map<stream_index_pair, int> unit_step_size_;
  std::vector<int> compression_params_;
  ob::FormatConvertFilter format_convert_filter_;

  std::map<stream_index_pair, bool> enable_stream_;
  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, image_transport::Publisher> image_publishers_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_publishers_;

  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> toggle_sensor_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> set_mirror_srv_;
  rclcpp::Service<GetInt32>::SharedPtr get_white_balance_srv_;
  rclcpp::Service<SetInt32>::SharedPtr set_white_balance_srv_;
  rclcpp::Service<GetInt32>::SharedPtr get_auto_white_balance_srv_;
  rclcpp::Service<SetBool>::SharedPtr set_auto_white_balance_srv_;
  rclcpp::Service<GetString>::SharedPtr get_sdk_version_srv_;
  rclcpp::Service<SetString>::SharedPtr switch_ir_camera_srv_;
  std::map<stream_index_pair, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr>
      set_auto_exposure_srv_;
  rclcpp::Service<GetDeviceInfo>::SharedPtr get_device_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_laser_enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ldp_enable_srv_;
  rclcpp::Service<orbbec_camera_msgs::srv::GetBool>::SharedPtr get_ldp_status_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_floor_enable_srv_;
  rclcpp::Service<SetInt32>::SharedPtr set_fan_work_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_sensors_srv_;

  bool publish_tf_ = false;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  bool enable_point_cloud_ = true;
  bool enable_colored_point_cloud_ = false;
  ob::PointCloudFilter point_cloud_filter_;
  sensor_msgs::msg::PointCloud2 point_cloud_msg_;

  rclcpp::Publisher<Extrinsics>::SharedPtr extrinsics_publisher_;
  bool enable_publish_extrinsic_ = false;
  orbbec_camera_msgs::msg::DeviceInfo device_info_;
  std::string point_cloud_qos_;
  std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
  std::unique_ptr<camera_info_manager::CameraInfoManager> ir_info_manager_ = nullptr;
  std::unique_ptr<camera_info_manager::CameraInfoManager> color_info_manager_ = nullptr;
  std::string color_info_url_;
  std::string ir_info_url_;
  std::optional<OBCameraParam> camera_param_;
  bool enable_d2c_viewer_ = false;
  std::unique_ptr<D2CViewer> d2c_viewer_ = nullptr;
  std::map<stream_index_pair, std::atomic_bool> save_images_;
  std::map<stream_index_pair, int> remaining_images_to_save_;
  std::atomic_bool save_point_cloud_{false};
  std::atomic_bool save_colored_point_cloud_{false};
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_images_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_point_cloud_srv_;
  bool enable_soft_filter_ = true;
  bool enable_color_auto_exposure_ = true;
  bool enable_ir_auto_exposure_ = true;
  int save_image_count_ = 100;
  std::queue<std::pair<std::string, cv::Mat>> image_queue_;
  std::atomic_bool run_save_image_thread_{false};
  std::shared_ptr<std::thread> save_image_thread_ = nullptr;
  std::condition_variable save_image_cv_;
  std::mutex save_image_mutex_;
  // Only for Gemini2 device
  bool enable_hardware_d2d_ = true;
  std::string depth_work_mode_;
  OBSyncMode sync_mode_ = OBSyncMode::OB_SYNC_MODE_CLOSE;
  std::string sync_mode_str_;
  int ir_trigger_signal_in_delay_ = 0;
  int rgb_trigger_signal_in_delay_ = 0;
  int device_trigger_signal_out_delay_ = 0;
  std::string depth_precision_str_;
  bool sync_signal_trigger_out_ = false;
  OB_DEPTH_PRECISION_LEVEL depth_precision_ = OB_PRECISION_0MM8;
};
}  // namespace orbbec_camera
