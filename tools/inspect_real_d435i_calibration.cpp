// Direct librealsense calibration audit for a connected Intel RealSense D435i.
//
// This intentionally queries the device through the public librealsense C++
// API.  It complements rs-enumerate-devices --calib_data and makes the
// calibration source part of the R1.5 evidence bundle.

#include <librealsense2/rs.hpp>

#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace {

std::string stream_key(const rs2::stream_profile& profile) {
  std::ostringstream key;
  key << static_cast<int>(profile.stream_type()) << ":"
      << profile.stream_index() << ":" << profile.fps() << ":"
      << static_cast<int>(profile.format());
  if (profile.stream_type() == RS2_STREAM_INFRARED ||
      profile.stream_type() == RS2_STREAM_DEPTH ||
      profile.stream_type() == RS2_STREAM_COLOR) {
    auto video = profile.as<rs2::video_stream_profile>();
    key << ":" << video.width() << "x" << video.height();
  }
  return key.str();
}

void print_motion_intrinsics(const std::string& name,
                             const rs2_motion_device_intrinsic& intrinsics) {
  std::cout << "motion_intrinsics " << name << "\n";
  std::cout << "  bias_variances " << std::setprecision(17)
            << intrinsics.bias_variances[0] << " "
            << intrinsics.bias_variances[1] << " "
            << intrinsics.bias_variances[2] << "\n";
  std::cout << "  noise_variances " << intrinsics.noise_variances[0] << " "
            << intrinsics.noise_variances[1] << " "
            << intrinsics.noise_variances[2] << "\n";
  std::cout << "  sensitivity\n";
  for (int row = 0; row < 3; ++row) {
    std::cout << "    " << intrinsics.data[row][0] << " "
              << intrinsics.data[row][1] << " "
              << intrinsics.data[row][2] << " "
              << intrinsics.data[row][3] << "\n";
  }
}

void print_extrinsics(const std::string& from, const std::string& to,
                      const rs2::stream_profile& source,
                      const rs2::stream_profile& target) {
  const auto extrinsics = source.get_extrinsics_to(target);
  std::cout << "extrinsics " << from << " -> " << to << "\n";
  std::cout << "  rotation\n";
  for (int row = 0; row < 3; ++row) {
    std::cout << "    " << extrinsics.rotation[row * 3] << " "
              << extrinsics.rotation[row * 3 + 1] << " "
              << extrinsics.rotation[row * 3 + 2] << "\n";
  }
  std::cout << "  translation " << std::setprecision(17)
            << extrinsics.translation[0] << " "
            << extrinsics.translation[1] << " "
            << extrinsics.translation[2] << "\n";
}

}  // namespace

int main() {
  try {
    rs2::context context;
    const auto devices = context.query_devices();
    if (devices.size() == 0) {
      std::cerr << "no_realsense_device\n";
      return 2;
    }

    const auto device = devices.front();
    std::cout << "name " << device.get_info(RS2_CAMERA_INFO_NAME) << "\n";
    std::cout << "serial "
              << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n";
    std::cout << "firmware "
              << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n";
    std::cout << "librealsense " << RS2_API_VERSION_STR << "\n";

    std::map<std::string, rs2::stream_profile> selected_profiles;
    std::set<std::string> emitted_intrinsics;
    for (const auto& sensor : device.query_sensors()) {
      for (const auto& profile : sensor.get_stream_profiles()) {
        const auto key = stream_key(profile);
        if (profile.stream_type() == RS2_STREAM_GYRO ||
            profile.stream_type() == RS2_STREAM_ACCEL) {
          if (emitted_intrinsics.insert(key).second) {
            const auto motion = profile.as<rs2::motion_stream_profile>();
            print_motion_intrinsics(key, motion.get_motion_intrinsics());
          }
        }
        if (profile.stream_type() == RS2_STREAM_GYRO &&
            profile.fps() == 200) {
          selected_profiles["gyro"] = profile;
        }
        if (profile.stream_type() == RS2_STREAM_INFRARED &&
            profile.stream_index() == 1 && profile.fps() == 30) {
          auto video = profile.as<rs2::video_stream_profile>();
          if (video.width() == 848 && video.height() == 480) {
            selected_profiles["infra1"] = profile;
          }
        }
      }
    }

    if (selected_profiles.count("gyro") && selected_profiles.count("infra1")) {
      print_extrinsics("gyro", "infra1", selected_profiles.at("gyro"),
                       selected_profiles.at("infra1"));
      print_extrinsics("infra1", "gyro", selected_profiles.at("infra1"),
                       selected_profiles.at("gyro"));
    } else {
      std::cout << "extrinsics_selection incomplete\n";
    }
    return 0;
  } catch (const rs2::error& error) {
    std::cerr << "librealsense_error " << error.get_failed_function() << ": "
              << error.what() << "\n";
    return 3;
  } catch (const std::exception& error) {
    std::cerr << "exception " << error.what() << "\n";
    return 4;
  }
}
