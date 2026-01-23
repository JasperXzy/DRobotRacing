#include <sstream>
#include "aurora900_ros2_device.h"
using namespace std;
using namespace deptrum;
Aurora900RosDevice::Aurora900RosDevice(std::shared_ptr<rclcpp::Node> node) : RosDevice(node) {
  if (params_.rgb_enable) {
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", 1);
    rgb_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info",
        1);
  }
  if (params_.ir_enable) {
    ir_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir/image_raw", 1);
  }
  if (params_.depth_enable) {
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 1);
  }
  if (params_.point_cloud_enable) {
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);
  }
  if (params_.depth_enable || params_.ir_enable || params_.point_cloud_enable) {
    ir_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "ir/camera_info",
        1);
  }
}
Aurora900RosDevice::~Aurora900RosDevice() {
  Stop();
}
int Aurora900RosDevice::Start() {
  if (running_ == true) {
    RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::Device already started", params_.boot_order);
    return 0;
  }

  if (device_ != nullptr) {
    camera_parameters_.Initialize(device_, params_.boot_order);
  }

  running_ = true;

  int status = 0;
  std::vector<deptrum::stream::StreamType> types;
  if (params_.rgbd_enable) {
    types.push_back(deptrum::stream::StreamType::kRgbdIr);
    if (params_.point_cloud_enable) {
      types.push_back(deptrum::stream::StreamType::kRgbdPointCloud);
    }
  } else {
    if (params_.rgb_enable) {
      types.push_back(deptrum::stream::StreamType::kRgb);
    }
    if (params_.depth_enable || params_.ir_enable) {
      types.push_back(deptrum::stream::StreamType::kDepthIr);
    }
    if (params_.point_cloud_enable) {
      types.push_back(deptrum::stream::StreamType::kPointCloud);
    }
  }

  if (types.size() != 0) {
    status = device_->CreateStream(streams_, types);
    if (0 != status) {
      RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::Create stream failed, error code %d", params_.boot_order, status);
      return status;
    }

    status = streams_->Start();
    if (0 != status) {
      RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::Start stream failed, error code %d", params_.boot_order, status);
      return status;
    }

    frame_publisher_thread_ = thread(&Aurora900RosDevice::FramePublisherThread, this);
  }

  return 0;
}

void Aurora900RosDevice::Stop() {
  RCLCPP_DEBUG(node_->get_logger(), "begin stop()");
  if (running_ == false) {
    RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::Cameras not started", params_.boot_order);
    return;
  }
  running_ = false;
  if (device_ != nullptr) {
    if (params_.heart_enable) {
      RCLCPP_DEBUG(node_->get_logger(), "close heart ...");
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device_)->StopHeartbeat();
    }
    // if (detector_thread_.joinable())
    // {
    //   detector_thread_.join();
    // }
    if (frame_publisher_thread_.joinable()) {
      frame_publisher_thread_.join();
    }

    if (streams_ != nullptr) {
      RCLCPP_DEBUG(node_->get_logger(), "stop stream");
      streams_->Stop();
      RCLCPP_DEBUG(node_->get_logger(), "destroy stream");
      device_->DestroyStream(streams_);
      streams_ = nullptr;
    }
  }
}

void Aurora900RosDevice::InitDevice() {
  int status = 0;
  std::vector<deptrum::DeviceInformation> device_list;
  status = deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);
  if (status != 0) {
    RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::GetDeviceList failed, error code %d", params_.boot_order, status);
  }
  int device_count = device_list.size();
  RCLCPP_DEBUG(node_->get_logger(), "device_count==========%d", device_count);
  if (device_count == 0) {
    // 等待设备插入
    unsigned int wait_num = 0;
    while (1) {
      wait_num++;
      if (hot_plug_num == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "wait device insert...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        status = deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);
        if (status != 0) {
          RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::GetDeviceList failed, error code %d", params_.boot_order, status);
        }
        if (device_list.size() > 0) {
          RCLCPP_INFO(node_->get_logger(), "has device insert");
          device_count = device_list.size();
          break;
        }
      }
      if (wait_num == 30) {
        RCLCPP_ERROR(node_->get_logger(), "device quit force with timeout......");
        RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::No deptrum device connected! It's going to quit...", params_.boot_order);
        rclcpp::shutdown();
        exit(-1);
      }
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::Found %d deptrum device", params_.boot_order, device_count);
  if (device_ != nullptr) {
    RCLCPP_WARN(node_->get_logger(), "BOOT_ORDER_%d::device_ is already init.", params_.boot_order);
  }

  if (params_.serial_number != "") {
    RCLCPP_INFO(node_->get_logger(), "Searching for device with serial number: %s", params_.serial_number.c_str());
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::No serial number provided: picking first device", params_.boot_order);
    // ROS_INFO_COND(device_count > 1,
    //               "BOOT_ORDER_%d::Multiple device connected! Openning first device.",
    //               params_.boot_order);
  }

  if (params_.usb_port_number.size()) {
    for (int i = 0; i < device_count; i++) {
      RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::%d deptrum device usb port %s", params_.boot_order, device_count, device_list[i].ir_camera.port_path.c_str());
    }
    device_count = 1;
  }

  int i = 0;
  for (; i < device_count; i++) {
    std::shared_ptr<deptrum::stream::Device> device = nullptr;
    if (params_.usb_port_number.size()) {
      RCLCPP_INFO(node_->get_logger(), "BOOT_ORDER_%d::USB port number:%s", params_.boot_order, params_.usb_port_number.c_str());
      device = deptrum::stream::DeviceManager::GetInstance()->CreateDeviceByUsbPort(
          params_.usb_port_number);
    } else {
      device = deptrum::stream::DeviceManager::GetInstance()->CreateDevice(device_list[i]);
    }
    if (device == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::Failed to create device", params_.boot_order);

      rclcpp::shutdown();
      exit(-1);
    }
    if (params_.stream_sdk_log_enable) {
      RCLCPP_DEBUG(node_->get_logger(), "open sdk log");
      deptrum::stream::DeviceManager::EnableLogging(params_.log_dir + "deptrum-stream.log",
                                                    params_.stream_sdk_log_enable);
    }

    deptrum::FrameMode ir_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameMode rgb_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameMode depth_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameDecodeMode ir_decode_mode{deptrum::FrameDecodeMode::kSoftwareDecode};
    deptrum::FrameDecodeMode rgb_decode_mode{deptrum::FrameDecodeMode::kSoftwareDecode};

    RCLCPP_INFO(node_->get_logger(), "BOOT_ORDER_%d::%d:device start opening", params_.boot_order, i);
    status = device->Open();
    if (status != 0) {
      device->Close();
      RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::open failed, error code %d", params_.boot_order, status);
      continue;
    }
    // after open device
    // for (int i = 1; i < 4; i++) {
    //   // ValidateFps(params_.ir_fps);
    //   status =
    //   dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetIrFps(params_.ir_fps); if
    //   (status != 0) {
    //     LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Ir fps failed, error code "
    //                << status << " num: " << i;
    //     continue;
    //   }

    //   // ValidateFps(params_.rgb_fps);
    //   status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetRgbFps(
    //       params_.rgb_fps);
    //   if (status != 0) {
    //     LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Rgb fps failed, error code "
    //                << status << " num: " << i;
    //     ;
    //     continue;
    //   }
    //   break;
    // }

    deptrum::FrameMode ir_mode_ref, rgb_mode_ref, depth_mode_ref;
    int num = 0;
    std::vector<std::tuple<deptrum::FrameMode, deptrum::FrameMode, deptrum::FrameMode>>
        device_resolution_vec;
    device->GetSupportedFrameMode(device_resolution_vec);
    if (device_resolution_vec.empty()) {
      device->Close();
      RCLCPP_ERROR(node_->get_logger(), "GetSupportedFrameMode is empty");
      rclcpp::shutdown();
      exit(-1);
    }
    RCLCPP_DEBUG(node_->get_logger(), "device supported frame_mode:");
    for (auto device_resolution : device_resolution_vec) {
      ir_mode_ref = std::get<0>(device_resolution);
      rgb_mode_ref = std::get<1>(device_resolution);
      depth_mode_ref = std::get<2>(device_resolution);
      RCLCPP_DEBUG(node_->get_logger(), "[%d]: ir_mode:=%d %s rgb_mode:=%d %s depth_mode:=%d %s",
                num++,
                int(ir_mode_ref), FrameModeToString(ir_mode_ref).c_str(),
                int(rgb_mode_ref), FrameModeToString(rgb_mode_ref).c_str(),
                int(depth_mode_ref), FrameModeToString(depth_mode_ref).c_str());
    }
    /*400device rgb version 2.2.0->resolution_mode_index->2   2.1.0->resolution_mode_index->0*/
    // dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetDeviceInfo(device_information_);
    // if ("2.2.0.0" == device_information_.rgb_firmware_version) {
    //   LOG(WARNING) << "检测到rgb firmware version
    //   is 2.2.0.0,rgb分辨率强制采用kRes640x480RgbJpeg"; params_.resolution_mode_index = 2;
    // }
    ir_mode = std::get<0>(device_resolution_vec[params_.resolution_mode_index]);
    rgb_mode = std::get<1>(device_resolution_vec[params_.resolution_mode_index]);
    RCLCPP_INFO(node_->get_logger(), "current set mode ir_mode:%d:%s,rgb_mode:%d:%s",
                (int) ir_mode, FrameModeToString(ir_mode).c_str(),
                (int) rgb_mode, FrameModeToString(rgb_mode).c_str());
    if (params_.output_compressed_rgb) {
      rgb_decode_mode = deptrum::FrameDecodeMode::kNoDecode;
    }

    status = device->SetMode(ir_mode, rgb_mode, depth_mode);
    if (status != 0) {
      RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::SetMode failed, error code %d", params_.boot_order, status);
      continue;
    }
    if (!SetDeviceParams(device)) {
      RCLCPP_WARN(node_->get_logger(), "set device params error");
    }

    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetIrFps(params_.ir_fps);
    if (status != 0) {
      RCLCPP_ERROR(node_->get_logger(), "set ir fps error,set it only 5 or 10 or 12 or 15 fps, try check it error_code=%d", status);
      rclcpp::shutdown();
      exit(-1);
    }

    deptrum::DeviceInformation device_information;
    deptrum::stream::DeviceDescription camera_information;
    device->GetDeviceInfo(device_information);
    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetDeviceInfo(
        camera_information);
    if (status != 0) {
      // ROS_WARN("BOOT_ORDER_%d::GetDeviceInfo failed, error code %x", params_.boot_order, status);
      RCLCPP_WARN(node_->get_logger(), "BOOT_ORDER_%d:GetDeviceInfo failed, error code %d", params_.boot_order, status);
      device->Close();
      continue;
    } else if (params_.serial_number == "" ||
               device_information.ir_camera.serial_number == params_.serial_number) {
      RCLCPP_DEBUG(node_->get_logger(), "BOOT_ORDER_%d::Device information: ", params_.boot_order);
      RCLCPP_DEBUG(node_->get_logger(), "\t sdk version: %s", camera_information.stream_sdk_version.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "\t device name: %s", camera_information.device_name.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "\t serial number: %s", camera_information.serial_num.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "\t firmware version: %s", camera_information.ir_firmware_version.c_str());

      if (device_ != nullptr) {
        // ROS_WARN("BOOT_ORDER_%d::Device Number: %d, device_ is already init.",
        //          params_.boot_order,
        //          i);
        RCLCPP_WARN(node_->get_logger(), "BOOT_ORDER_%d::Device Number: %d, device_ is already init.", params_.boot_order, i);
      }
      device_ = device;
      break;
    } else {
      device->Close();
      continue;
    }
  }

  if (!device_) {
    // ROS_ERROR("BOOT_ORDER_%d::All %d devices have attempted to connect, Failed to open device",
    //           params_.boot_order,
    //           device_count);
    RCLCPP_ERROR(node_->get_logger(), "BOOT_ORDER_%d::All %d devices have attempted to connect, Failed to open device ", params_.boot_order, device_count);
    rclcpp::shutdown();
    exit(-1);
  }
}

bool Aurora900RosDevice::SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) {
  int status = 0;

  if (params_.update_file_path != "") {
    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->Upgrade(
        params_.update_file_path,
        upgrade_handle_);
    if (status != 0) {
      RCLCPP_WARN(node_->get_logger(), "update firmware failed, error code %d", status);
      exit(-1);
    } else {
      RCLCPP_INFO(node_->get_logger(), "update firmware succeed");
      rclcpp::shutdown();
      exit(0);
    }
  }

  status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->EnableDepthIrMirroring(
      params_.depth_ir_mirroring);
  if (status != 0) {
    // ROS_WARN("EnableDepthIrMirroring failed, error code %x", status);
    RCLCPP_WARN(node_->get_logger(), "EnableDepthIrMirroring failed, error code %d", status);
    return false;
  }
  if (params_.exposure_enable) {
    if (params_.exposure_time < 1 || params_.exposure_time > 31)
      RCLCPP_WARN(node_->get_logger(), "current exposure time is %d please input [1-31],单位为0.1ms", params_.exposure_time);
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAutoExposure(false);
      if (status != 0) {
        RCLCPP_WARN(node_->get_logger(), "SwitchAutoExposure failed, error code %d", status);
        return false;
      }
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetExposure(
          kIrCamera,
          params_.exposure_time);
      if (status != 0) {
        RCLCPP_WARN(node_->get_logger(), "SetExposure failed, error code %d", status);
        return false;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "set exposure value %d succeed", params_.exposure_time);
      }
      // int current_exposure_value = 0;
      // dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetExposure(kIrCamera,
      //                                                                       current_exposure_value);
      // LOG(INFO) << "current exposure value is " << current_exposure_value;
    }
  } else {
    if (device->GetDeviceName() == "Aurora930") {
      RCLCPP_DEBUG(node_->get_logger(), "Switch Auto Exposure");
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAutoExposure(true);
    }
  }
  if (params_.gain_enable) {
    // ValidateGain(params_.gain_value); 10-160
    if (params_.gain_value < 10 || params_.gain_value >= 160)
      RCLCPP_WARN(node_->get_logger(), "current gain value is %d please input [10-160), curren is not valid", params_.gain_value);
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetGain(
          kIrCamera,
          params_.gain_value);
      if (status != 0) {
        RCLCPP_WARN(node_->get_logger(), "SetGain failed, error code %d", status);
        return false;
      }
      uint32_t current_gain_value = 0;
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetGain(kIrCamera,
                                                                        current_gain_value);
      RCLCPP_DEBUG(node_->get_logger(), "current gain value is %d", current_gain_value);
    }
  }
  if (device->GetDeviceName() == "Aurora930") {
    if (params_.threshold_size < 30 || params_.threshold_size > 400)
      RCLCPP_WARN(node_->get_logger(), "current threshold size is %d please input [30-400], curren is not valid", params_.threshold_size);
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetRemoveFilterSize(
          params_.threshold_size);
      if (status != 0) {
        RCLCPP_WARN(node_->get_logger(), "SetRemoveFilterSize failed, error code %d", status);
        return false;
      }
    }
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->DepthCorrection(
        params_.depth_correction);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAlignedMode(params_.align_mode);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->FilterOutRangeDepthMap(
        params_.minimum_filter_depth_value,
        params_.maximum_filter_depth_value);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetLaserDriver(params_.laser_power);

    deptrum::stream::HeartbeatParam heartbeat_param;
    if (params_.heart_enable) {
      heartbeat_param.is_enabled = 1;  // open heart beat
    } else {
      heartbeat_param.is_enabled = 0;  // close heart beat
    }
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetHeartbeat(
        heartbeat_param,
        [this](int result) {
          if (0 == result) {
            RCLCPP_WARN(node_->get_logger(), "heartbeat failed!!!");
          }
        });
  }

  return true;
}
