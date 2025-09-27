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

#include "orbbec_camera/utils.h"
namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion, int width) {
  sensor_msgs::msg::CameraInfo info;
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info.width = intrinsic.width;
  info.height = intrinsic.height;
  info.d.resize(5, 0.0);
  info.d[0] = distortion.k1;
  info.d[1] = distortion.k2;
  info.d[2] = distortion.k3;
  info.d[3] = distortion.k4;
  info.d[4] = distortion.k5;

  info.k.fill(0.0);
  info.k[0] = intrinsic.fx;
  info.k[2] = intrinsic.cx;
  info.k[4] = intrinsic.fy;
  info.k[5] = intrinsic.cy;
  info.k[8] = 1.0;

  info.r.fill(0.0);
  info.r[0] = 1;
  info.r[4] = 1;
  info.r[8] = 1;

  info.p.fill(0.0);
  info.p[0] = info.k[0];
  info.p[2] = info.k[2];
  info.p[5] = info.k[4];
  info.p[6] = info.k[5];
  info.p[10] = 1.0;
  double scaling = static_cast<double>(width) / 640;
  info.k[0] *= scaling;  // fx
  info.k[2] *= scaling;  // cx
  info.k[4] *= scaling;  // fy
  info.k[5] *= scaling;  // cy
  info.p[0] *= scaling;  // fx
  info.p[2] *= scaling;  // cx
  info.p[5] *= scaling;  // fy
  info.p[6] *= scaling;  // cy
  return info;
}

void saveRGBPointsToPly(const std::shared_ptr<ob::Frame> &frame, const std::string &fileName) {
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  std::shared_ptr<int> fp_guard(nullptr, [&fp](int *) {
    fflush(fp);
    fclose(fp);
  });
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  const auto *points = (OBColorPoint *)frame->data();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", points[i].x, points[i].y, points[i].z,
            (int)points[i].r, (int)points[i].g, (int)points[i].b);
  }
}

void savePointsToPly(const std::shared_ptr<ob::Frame> &frame, const std::string &fileName) {
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  std::shared_ptr<int> fp_guard(nullptr, [&fp](int *) {
    fflush(fp);
    fclose(fp);
  });
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  const auto *points = (OBPoint *)frame->data();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f\n", points[i].x, points[i].y, points[i].z);
  }
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7], rotation[2],
      rotation[5], rotation[8];
  Eigen::Quaternionf q(m);
  return {q.x(), q.y(), q.z(), q.w()};
}
std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs) {
  auto depth_intrinsic = rhs.depthIntrinsic;
  auto rgb_intrinsic = rhs.rgbIntrinsic;
  os << "=====depth intrinsic=====\n";
  os << "fx : " << depth_intrinsic.fx << "\n";
  os << "fy : " << depth_intrinsic.fy << "\n";
  os << "cx : " << depth_intrinsic.cx << "\n";
  os << "cy : " << depth_intrinsic.cy << "\n";
  os << "width : " << depth_intrinsic.width << "\n";
  os << "height : " << depth_intrinsic.height << "\n";
  os << "=====rgb intrinsic=====\n";
  os << "fx : " << rgb_intrinsic.fx << "\n";
  os << "fy : " << rgb_intrinsic.fy << "\n";
  os << "cx : " << rgb_intrinsic.cx << "\n";
  os << "cy : " << rgb_intrinsic.cy << "\n";
  os << "width : " << rgb_intrinsic.width << "\n";
  os << "height : " << rgb_intrinsic.height << "\n";
  return os;
}

orbbec_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics,
                                                      const std::string &frame_id) {
  orbbec_camera_msgs::msg::Extrinsics msg;
  for (int i = 0; i < 9; ++i) {
    msg.rotation[i] = extrinsics.rot[i];
    if (i < 3) {
      msg.translation[i] = extrinsics.trans[i];
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

rclcpp::Time frameTimeStampToROSTime(uint64_t ms) {
  auto total = static_cast<uint64_t>(ms * 1e6);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  rclcpp::Time stamp(sec, nano_sec);
  return stamp;
}
std::string getObSDKVersion() {
  std::string major = std::to_string(ob::Version::getMajor());
  std::string minor = std::to_string(ob::Version::getMinor());
  std::string patch = std::to_string(ob::Version::getPatch());
  std::string version = major + "." + minor + "." + patch;
  return version;
}
OBFormat OBFormatFromString(const std::string &format) {
  std::string fixed_format;
  std::transform(format.begin(), format.end(), std::back_inserter(fixed_format),
                 [](const auto ch) { return std::isalpha(ch) ? toupper(ch) : ch; });
  if (fixed_format == "YUYV") {
    return OB_FORMAT_YUYV;
  } else if (fixed_format == "MJPG") {
    return OB_FORMAT_MJPG;
  } else if (fixed_format == "MJPEG") {
    return OB_FORMAT_MJPEG;
  } else if (fixed_format == "YUYV2") {
    return OB_FORMAT_YUY2;
  } else if (fixed_format == "UYVY") {
    return OB_FORMAT_UYVY;
  } else if (fixed_format == "NV12") {
    return OB_FORMAT_NV12;
  } else if (fixed_format == "NV21") {
    return OB_FORMAT_NV21;
  } else if (fixed_format == "H264") {
    return OB_FORMAT_H264;
  } else if (fixed_format == "H265") {
    return OB_FORMAT_H265;
  } else if (fixed_format == "Y16") {
    return OB_FORMAT_Y16;
  } else if (fixed_format == "Y8") {
    return OB_FORMAT_Y8;
  } else if (fixed_format == "Y10") {
    return OB_FORMAT_Y10;
  } else if (fixed_format == "Y11") {
    return OB_FORMAT_Y11;
  } else if (fixed_format == "Y12") {
    return OB_FORMAT_Y12;
  } else if (fixed_format == "GRAY") {
    return OB_FORMAT_GRAY;
  } else if (fixed_format == "HEVC") {
    return OB_FORMAT_HEVC;
  } else if (fixed_format == "I420") {
    return OB_FORMAT_I420;
  } else if (fixed_format == "ACCEL") {
    return OB_FORMAT_ACCEL;
  } else if (fixed_format == "GYRO") {
    return OB_FORMAT_GYRO;
  } else if (fixed_format == "POINT") {
    return OB_FORMAT_POINT;
  } else if (fixed_format == "RGB_POINT") {
    return OB_FORMAT_RGB_POINT;
  } else if (fixed_format == "REL") {
    return OB_FORMAT_RLE;
  } else if (fixed_format == "RGB888" || fixed_format == "RGB") {
    return OB_FORMAT_RGB888;
  } else if (fixed_format == "BGR") {
    return OB_FORMAT_BGR;
  } else if (fixed_format == "Y14") {
    return OB_FORMAT_Y14;
  } else {
    return OB_FORMAT_UNKNOWN;
  }
}

std::string ObDeviceTypeToString(const OBDeviceType &type) {
  switch (type) {
    case OBDeviceType::OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA:
      return "structured light binocular camera";
    case OBDeviceType::OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA:
      return "structured light monocular camera";
    case OBDeviceType::OB_TOF_CAMERA:
      return "tof camera";
  }
  return "unknown technology camera";
}

rmw_qos_profile_t getRMWQosProfileFromString(const std::string &str_qos) {
  std::string upper_str_qos = str_qos;
  std::transform(upper_str_qos.begin(), upper_str_qos.end(), upper_str_qos.begin(), ::toupper);
  if (upper_str_qos == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  } else if (upper_str_qos == "DEFAULT") {
    return rmw_qos_profile_default;
  } else if (upper_str_qos == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  } else if (upper_str_qos == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  } else if (upper_str_qos == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  } else if (upper_str_qos == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("astra_camera"),
                        "Invalid QoS profile: " << upper_str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}
bool isOpenNIDevice(int pid) {
  static const std::vector<int> OPENNI_DEVICE_PIDS = {
      0x0300, 0x0301, 0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0407, 0x0601, 0x060b,
      0x060e, 0x060f, 0x0610, 0x0613, 0x0614, 0x0616, 0x0617, 0x0618, 0x061b, 0x062b,
      0x062c, 0x062d, 0x0632, 0x0633, 0x0634, 0x0635, 0x0636, 0x0637, 0x0638, 0x0639,
      0x063a, 0x0650, 0x0651, 0x0654, 0x0655, 0x0656, 0x0657, 0x0658, 0x0659, 0x065a,
      0x065b, 0x065c, 0x065d, 0x0698, 0x0699, 0x069a};

  for (const auto &pid_openni : OPENNI_DEVICE_PIDS) {
    if (pid == pid_openni) {
      return true;
    }
  }
  return false;
}

OB_DEPTH_PRECISION_LEVEL depthPrecisionLevelFromString(
    const std::string &depth_precision_level_str) {
  if (depth_precision_level_str == "1mm") {
    return OB_PRECISION_1MM;
  } else if (depth_precision_level_str == "0.8mm") {
    return OB_PRECISION_0MM8;
  } else if (depth_precision_level_str == "0.4mm") {
    return OB_PRECISION_0MM4;
  } else if (depth_precision_level_str == "0.2mm") {
    return OB_PRECISION_0MM2;
  } else if (depth_precision_level_str == "0.1mm") {
    return OB_PRECISION_0MM1;
  } else {
    return OB_PRECISION_0MM8;
  }
}

OBSyncMode OBSyncModeFromString(const std::string &mode) {
  if (mode == "CLOSE") {
    return OBSyncMode::OB_SYNC_MODE_CLOSE;
  } else if (mode == "PRIMARY") {
    return OBSyncMode::OB_SYNC_MODE_PRIMARY;
  } else if (mode == "STANDALONE") {
    return OBSyncMode::OB_SYNC_MODE_STANDALONE;
  } else if (mode == "SECONDARY") {
    return OBSyncMode::OB_SYNC_MODE_SECONDARY;
  } else if (mode == "PRIMARY_MCU_TRIGGER") {
    return OBSyncMode::OB_SYNC_MODE_PRIMARY_MCU_TRIGGER;
  } else if (mode == "PRIMARY_IR_TRIGGER") {
    return OBSyncMode::OB_SYNC_MODE_PRIMARY_IR_TRIGGER;
  } else if (mode == "PRIMARY_SOFT_TRIGGER") {
    return OBSyncMode::OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER;
  } else if (mode == "SECONDARY_SOFT_TRIGGER") {
    return OBSyncMode::OB_SYNC_MODE_SECONDARY_SOFT_TRIGGER;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), "Unknown OBSyncMode: " << mode);
    return OBSyncMode::OB_SYNC_MODE_CLOSE;
  }
}

}  // namespace orbbec_camera
