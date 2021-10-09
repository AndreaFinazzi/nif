/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearican Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearican Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/avt_vimba_api.h>
#include <avt_vimba_camera/avt_vimba_camera.h>

// ros1 #include <ros/ros.h>
// ros1 #include <ros/console.h>
#include "rclcpp/logger.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging_macros.h"

#include <signal.h>
using AVT::VmbAPI::FeaturePtr;
using AVT::VmbAPI::FeaturePtrVector;

namespace avt_vimba_camera {

static const char* TriggerMode[] = {
    "Freerun", "FixedRate", "Software", "Line1", "Line2", "Line3", "Line4"};

static const char* FeatureDataType[] = {
    "Unknown", "int", "float", "enum", "string", "bool"};

#if 0  // ros1: needed?
static const char* AutoMode[] = {
  "Off",
  "Once",
  "Continuous"};
static const char* AcquisitionMode[] = {
  "Continuous",
  "SingleFrame",
  "MultiFrame",
  "Recorder"};
static const char* PixelFormatMode[] = {
  "Mono8",
  "Mono10",
  "Mono10Packed",
  "Mono12",
  "Mono12Packed",
  "BayerGR8",
  "BayerRG8",
  "BayerGB8",
  "BayerBG8",
  "BayerGR10",
  "BayerRG10",
  "BayerGB10",
  "BayerBG10",
  "BayerGR12",
  "BayerRG12",
  "BayerGB12",
  "BayerBG12",
  "BayerGR10Packed",
  "BayerRG10Packed",
  "BayerGB10Packed",
  "BayerBG10Packed",
  "BayerGR12Packed",
  "BayerRG12Packed",
  "BayerGB12Packed",
  "BayerBG12Packed",
  "RGB8Packed",
  "BGR8Packed"};
static const char* BalanceRatioMode[] = {
  "Red",
  "Blue"};

static const char* State[] = {
  "Opening",
  "Idle",
  "Camera not found",
  "Format error",
  "Error",
  "Ok"};
#endif // 0

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  dummy |= 0;
  keepRunning = 0;
}

// ros1 AvtVimbaCamera::AvtVimbaCamera() :
// AvtVimbaCamera(ros::this_node::getName().c_str()) {}
AvtVimbaCamera::AvtVimbaCamera() : AvtVimbaCamera("avt_vimba_camera") {}

AvtVimbaCamera::AvtVimbaCamera(std::string name) {
  // Init global variables
  opened_ = false;    // camera connected to the api
  streaming_ = false; // capturing frames
  on_init_ = true;    // on initialization phase
  // show_debug_prints_ = false;
  show_debug_prints_ = true;
  name_ = name;

  camera_state_ = OPENING;

  // ros1 updater_.setHardwareID("unknown");
  // ros1 updater_.add(name_, this, &AvtVimbaCamera::getCurrentState);
  // ros1 updater_.update();
}

void AvtVimbaCamera::start(std::string ip_str,
                           std::string guid_str,
                           bool debug_prints) {
  if (opened_)
    return;
  show_debug_prints_ = debug_prints;
  // ros1 updater_.broadcast(0, "Starting device with IP:" + ip_str + " or
  // GUID:" + guid_str);
  std::cout << "Starting device with IP:" << ip_str << " or GUID:" << guid_str
            << std::endl;

  // Determine which camera to use. Try IP first
  if (!ip_str.empty()) {
    diagnostic_msg_ = "Trying to open camera by IP: " + ip_str;
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "Trying to open camera by IP: " << ip_str);

    vimba_camera_ptr_ = openCamera(ip_str);
    if (!vimba_camera_ptr_) {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "Camera pointer is empty. Returning...");
      return;
    }
    // ros1 updater_.setHardwareID(ip_str);
    guid_ = ip_str;
    // If both guid and IP are available, open by IP and check guid
    if (!guid_str.empty()) {
      std::string cam_guid_str;
      vimba_camera_ptr_->GetSerialNumber(cam_guid_str);
      if (!vimba_camera_ptr_) {
        RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                           "Camera pointer is empty. Returning...");
        return;
      }
      assert(cam_guid_str == guid_str);
      // ros1 updater_.setHardwareID(guid_str);
      guid_ = guid_str;
      diagnostic_msg_ =
          "GUID " + cam_guid_str + " matches for camera with IP: " + ip_str;
      RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                         "GUID " << cam_guid_str
                                 << " matches for camera with IP: " << ip_str);
    }
  } else if (!guid_str.empty()) {
    // Only guid available
    diagnostic_msg_ = "Trying to open camera by ID: " + guid_str;
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "Trying to open camera by ID: " << guid_str);
    vimba_camera_ptr_ = openCamera(guid_str);
    // ros1 updater_.setHardwareID(guid_str);
    guid_ = guid_str;
  } else {
    // No identifying info (GUID and IP) are available
    diagnostic_msg_ =
        "Can't connect to the camera: at least GUID or IP need to be set.";
    RCLCPP_ERROR(
        node_handle_->get_logger(),
        "Can't connect to the camera: at least GUID or IP need to be set.");
    camera_state_ = ERROR;
  }
  // ros1 updater_.update();

  // From the SynchronousGrab API example:
  // TODO Set the GeV packet size to the highest possible value
  VmbInterfaceType cam_int_type;
  vimba_camera_ptr_->GetInterfaceType(cam_int_type);
  if (cam_int_type == VmbInterfaceEthernet) {
    runCommand("GVSPAdjustPacketSize");
  }

  std::string trigger_source;
  getFeatureValue("TriggerSource", trigger_source);
  int trigger_source_int = getTriggerModeInt(trigger_source);

  if (trigger_source_int == Freerun || trigger_source_int == FixedRate ||
      trigger_source_int == SyncIn1) {
    // Create a frame observer for this camera
    // ros1 SP_SET(frame_obs_ptr_, new FrameObserver(vimba_camera_ptr_,
    // boost::bind(&avt_vimba_camera::AvtVimbaCamera::frameCallback, this,
    // _1)));
    SP_SET(frame_obs_ptr_,
           new FrameObserver(
               vimba_camera_ptr_,
               std::bind(&avt_vimba_camera::AvtVimbaCamera::frameCallback,
                         this,
                         std::placeholders::_1)));
    camera_state_ = IDLE;
  } else {
    diagnostic_msg_ = "Trigger mode " +
        std::string(TriggerMode[trigger_source_int]) + " not implemented.";
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "Trigger mode " << TriggerMode[trigger_source_int]
                                        << " not implemented.");
    camera_state_ = ERROR;
  }
  // ros1 updater_.update();
}

void AvtVimbaCamera::startImaging(void) {
  if (!streaming_) {
    // Start streaming
    VmbErrorType err = vimba_camera_ptr_->StartContinuousImageAcquisition(
        5, // num_frames_,
        IFrameObserverPtr(frame_obs_ptr_));
    if (VmbErrorSuccess == err) {
      diagnostic_msg_ = "Starting continuous image acquisition";
      RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                         "[" << name_
                             << "]: Starting continuous image acquisition...("
                             << frame_id_ << ")");
      streaming_ = true;
      camera_state_ = OK;
    } else {
      diagnostic_msg_ =
          "Could not start continuous image acquisition. Error: " +
          api_.errorCodeToMessage(err);
      RCLCPP_ERROR_STREAM(
          node_handle_->get_logger(),
          "[" << name_ << "]: Could not start continuous image acquisition("
              << frame_id_ << "). "
              << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  } else {
    RCLCPP_WARN_STREAM(
        node_handle_->get_logger(),
        "Start imaging called, but the camera is already imaging(" << frame_id_
                                                                   << ").");
  }
  // ros1 updater_.update();
}

void AvtVimbaCamera::stopImaging(void) {
  if (streaming_ || on_init_) {
    VmbErrorType err = vimba_camera_ptr_->StopContinuousImageAcquisition();
    if (VmbErrorSuccess == err) {
      diagnostic_msg_ = "Acquisition stopped";
      RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                         "[" << name_ << "]: Acquisition stoppped... ("
                             << frame_id_ << ")");
      streaming_ = false;
      camera_state_ = IDLE;
    } else {
      diagnostic_msg_ = "Could not stop image acquisition. Error: " +
          api_.errorCodeToMessage(err);
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                          "[" << name_
                              << "]: Could not stop image acquisition ("
                              << frame_id_ << ")."
                              << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  } else {
    RCLCPP_WARN_STREAM(
        node_handle_->get_logger(),
        "Stop imaging called, but the camera is already stopped (" << frame_id_
                                                                   << ").");
  }
  // ros1 updater_.update();
}

void AvtVimbaCamera::updateConfig(Config& config) {
  // ros1 boost::mutex::scoped_lock lock(config_mutex_);

  frame_id_ = config.frame_id_;

  if (streaming_) {
    stopImaging();
    // ros1 ros::Duration(0.5).sleep(); // sleep for half a second
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (on_init_) {
    config_ = config;
  }
  diagnostic_msg_ = "Updating configuration";
  if (show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "Updating configuration for camera "
                           << config.frame_id_);
  }
  updateExposureConfig(config);
  updateGainConfig(config);
  updateWhiteBalanceConfig(config);
  updateImageModeConfig(config);
  updateROIConfig(config);
  updateBandwidthConfig(config);
  updateGPIOConfig(config);
  updatePtpModeConfig(config);
  updatePixelFormatConfig(config);
  updateAcquisitionConfig(config);
  // NOTE : Iris config is not supported by the Mako-G
  // updateIrisConfig(config);
  config_ = config;

  if (on_init_) {
    on_init_ = false;
  }

  startImaging();
  ptpSyncTrigger();
}

void AvtVimbaCamera::stop() {
  if (!opened_)
    return;
  vimba_camera_ptr_->Close();
  opened_ = false;
}

CameraPtr AvtVimbaCamera::openCamera(std::string id_str) {
  // Details:   The ID might be one of the following:
  //            "IP:169.254.12.13",
  //            "MAC:000f31000001",
  //            or a plain serial number: "1234567890".

  CameraPtr camera;
  VimbaSystem& vimba_system(VimbaSystem::GetInstance());

  // set handler to catch ctrl+c presses
  sighandler_t oldHandler = signal(SIGINT, intHandler);

  // get camera
  VmbErrorType err = vimba_system.GetCameraByID(id_str.c_str(), camera);
  while (err != VmbErrorSuccess) {
    if (keepRunning) {
      RCLCPP_WARN(node_handle_->get_logger(),
                  "Could not get camera. Retrying every two seconds...");
      err = vimba_system.GetCameraByID(id_str.c_str(), camera);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                          "[" << name_ << "]: Could not get camera " << id_str
                              << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  // open camera
  err = camera->Open(VmbAccessModeFull);
  while (err != VmbErrorSuccess && keepRunning) {
    if (keepRunning) {
      RCLCPP_WARN(node_handle_->get_logger(),
                  "Could not open camera. Retrying every two seconds...");
      err = camera->Open(VmbAccessModeFull);
      // ros1 ros::Duration(2.0).sleep();
      std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                          "[" << name_ << "]: Could not open camera " << id_str
                              << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  // set previous handler back
  signal(SIGINT, oldHandler);

  std::string cam_id, cam_name, cam_model, cam_sn, cam_int_id;
  VmbInterfaceType cam_int_type;
  VmbAccessModeType accessMode; // = VmbAccessModeNone;
  camera->GetID(cam_id);
  camera->GetName(cam_name);
  camera->GetModel(cam_model);
  camera->GetSerialNumber(cam_sn);
  camera->GetInterfaceID(cam_int_id);
  camera->GetInterfaceType(cam_int_type);
  err = camera->GetPermittedAccess(accessMode);

  if (show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "[" << name_ << "]: Opened camera with"
            << "\n\t\t * Name     : " << cam_name << "\n\t\t * Model    : "
            << cam_model << "\n\t\t * ID       : " << cam_id
            << "\n\t\t * S/N      : " << cam_sn
            << "\n\t\t * Itf. ID  : " << cam_int_id
            << "\n\t\t * Itf. Type: " << interfaceToString(cam_int_type)
            << "\n\t\t * Access   : " << accessModeToString(accessMode));
  }

  // ros1 ros::Duration(2.0).sleep();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ros1 printAllCameraFeatures(camera);
  opened_ = true;
  camera_state_ = IDLE;
  return camera;
}

void AvtVimbaCamera::frameCallback(const FramePtr vimba_frame_ptr) {
  // ros1 boost::mutex::scoped_lock lock(config_mutex_);

  camera_state_ = OK;
  diagnostic_msg_ = "Camera operating normally";

  // Call the callback implemented by other classes
  // ros1 boost::thread thread_callback = boost::thread(userFrameCallback,
  // vimba_frame_ptr); ros1 thread_callback.join();
  userFrameCallback(vimba_frame_ptr);

  // ros1 updater_.update();
}

double AvtVimbaCamera::getDeviceTemp(void) {
  double temp = -1.0;
  if (setFeatureValue("DeviceTemperatureSelector", "Main")) {
    getFeatureValue("DeviceTemperature", temp);
  }
  return temp;
}

bool AvtVimbaCamera::resetTimestamp(void) {
  return runCommand("GevTimestampControlReset");
}

double AvtVimbaCamera::getTimestamp(void) {
  double timestamp = -1.0;
  if (runCommand("GevTimestampControlLatch")) {
    VmbInt64_t freq, ticks;
    getFeatureValue("GevTimestampTickFrequency", freq);
    getFeatureValue("GevTimestampValue", ticks);
    timestamp = ((double)ticks) / ((double)freq);
  }
  return timestamp;
}

// Template function to GET a feature value from the camera
template <typename T>
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, T& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
      vimba_feature_ptr->GetDataType(data_type);
      if (VmbErrorSuccess != err) {
        std::cout << "[Could not get feature Data Type. Error code: " << err
                  << "]" << std::endl;
      } else {
        std::string strValue;
        switch (data_type) {
        case VmbFeatureDataBool:
          bool bValue;
          err = vimba_feature_ptr->GetValue(bValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(bValue);
          }
          break;
        case VmbFeatureDataFloat:
          double fValue;
          err = vimba_feature_ptr->GetValue(fValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(fValue);
          }
          break;
        case VmbFeatureDataInt:
          VmbInt64_t nValue;
          err = vimba_feature_ptr->GetValue(nValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(nValue);
          }
          break;
        default:
          break;
        }
        if (VmbErrorSuccess != err) {
          RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                             "Could not get feature value. Error code: "
                                 << api_.errorCodeToMessage(err));
        }
      }
    } else {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "[" << name_ << "]: Feature " << feature_str
                             << " is not readable.");
    }
  } else {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "[" << name_ << "]: Could not get feature "
                           << feature_str);
  }
  if (show_debug_prints_)
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "Asking for feature " << feature_str << " with datatype "
                                             << FeatureDataType[data_type]
                                             << " and value " << val);
  return (VmbErrorSuccess == err);
}

// Function to GET a feature value from the camera, overloaded to strings
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str,
                                     std::string& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
      vimba_feature_ptr->GetDataType(data_type);
      if (VmbErrorSuccess != err) {
        std::cout << "[Could not get feature Data Type. Error code: " << err
                  << "]" << std::endl;
      } else {
        std::string strValue;
        switch (data_type) {
        case VmbFeatureDataEnum:
          err = vimba_feature_ptr->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            val = strValue;
          }
          break;
        case VmbFeatureDataString:
          err = vimba_feature_ptr->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            val = strValue;
          }
          break;
        default:
          break;
        }
        if (VmbErrorSuccess != err) {
          RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                             "Could not get feature value. Error code: "
                                 << api_.errorCodeToMessage(err));
        }
      }
    } else {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "[" << name_ << "]: Feature " << feature_str
                             << " is not readable.");
    }
  } else {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "[" << name_ << "]: Could not get feature "
                           << feature_str);
  }
  if (show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "Asking for feature " << feature_str << " with datatype "
                                             << FeatureDataType[data_type]
                                             << " and value " << val);
  }
  return (VmbErrorSuccess == err);
}

// Template function to SET a feature value from the camera
template <typename T>
bool AvtVimbaCamera::setFeatureValue(const std::string& feature_str,
                                     const T& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool writable;
    err = vimba_feature_ptr->IsWritable(writable);
    if (VmbErrorSuccess == err) {
      if (writable) {
        if (show_debug_prints_)
          RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                             "Setting feature " << feature_str << " value "
                                                << val);
        VmbFeatureDataType data_type;
        err = vimba_feature_ptr->GetDataType(data_type);
        if (VmbErrorSuccess == err) {
          if (data_type == VmbFeatureDataEnum) {
            bool available;
            err = vimba_feature_ptr->IsValueAvailable(val, available);
            if (VmbErrorSuccess == err) {
              if (available) {
                err = vimba_feature_ptr->SetValue(val);
              } else {
                RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                                   "[" << name_ << "]: Feature " << feature_str
                                       << " is available now.");
              }
            } else {
              RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                                 "[" << name_ << "]: Feature " << feature_str
                                     << ": value unavailable\n\tERROR "
                                     << api_.errorCodeToMessage(err));
            }
          } else {
            err = vimba_feature_ptr->SetValue(val);
          }
        } else {
          RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                             "[" << name_ << "]: Feature " << feature_str
                                 << ": Bad data type\n\tERROR "
                                 << api_.errorCodeToMessage(err));
        }
      } else {
        RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                           "[" << name_ << "]: Feature " << feature_str
                               << " is not writable.");
      }
    } else {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "[" << name_ << "]: Feature " << feature_str
                             << ": ERROR " << api_.errorCodeToMessage(err));
    }
  } else {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "[" << name_ << "]: Could not get feature "
                           << feature_str
                           << "\n Error: " << api_.errorCodeToMessage(err));
  }
  return (VmbErrorSuccess == err);
}

// Template function to RUN a command
bool AvtVimbaCamera::runCommand(const std::string& command_str) {
  FeaturePtr feature_ptr;
  VmbErrorType err =
      vimba_camera_ptr_->GetFeatureByName(command_str.c_str(), feature_ptr);
  if (VmbErrorSuccess == err) {
    err = feature_ptr->RunCommand();
    if (VmbErrorSuccess == err) {
      bool is_command_done = false;
      do {
        err = feature_ptr->IsCommandDone(is_command_done);
        if (VmbErrorSuccess != err) {
          break;
        }
        if (show_debug_prints_) {
          /* INFO_S_T */ std::cout
              << "Waiting for command "
              // ros1 << command_str.c_str() << "..." << std::endl;
              << "..." << std::endl;
        }
      } while (false == is_command_done);
      if (show_debug_prints_)
        RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                           "Command " << command_str.c_str() << " done!");
      return true;
    } else {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "[" << name_ << "]: Could not run command "
                             << command_str
                             << ". Error: " << api_.errorCodeToMessage(err));
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "[" << name_ << "]: Could not get feature command "
                           << command_str
                           << ". Error: " << api_.errorCodeToMessage(err));
    return false;
  }
}

std::string AvtVimbaCamera::interfaceToString(VmbInterfaceType interfaceType) {
  switch (interfaceType) {
  case VmbInterfaceFirewire:
    return "FireWire";
    break;
  case VmbInterfaceEthernet:
    return "GigE";
    break;
  case VmbInterfaceUsb:
    return "USB";
    break;
  default:
    return "Unknown";
  }
}

std::string AvtVimbaCamera::accessModeToString(VmbAccessModeType modeType) {
  std::string s;
  if (modeType & VmbAccessModeFull)
    s = std::string("Read and write access");
  else if (modeType & VmbAccessModeRead)
    s = std::string("Only read access");
  else if (modeType & VmbAccessModeConfig)
    s = std::string("Device configuration access");
  else if (modeType & VmbAccessModeLite)
    s = std::string(
        "Device read/write access without feature access (only addresses)");
  else if (modeType & VmbAccessModeNone)
    s = std::string("No access");
  return s;
}

int AvtVimbaCamera::getTriggerModeInt(std::string mode_str) {
  int mode;
  if (mode_str == TriggerMode[Freerun]) {
    mode = Freerun;
  } else if (mode_str == TriggerMode[FixedRate]) {
    mode = FixedRate;
  } else if (mode_str == TriggerMode[Software]) {
    mode = Software;
  } else if (mode_str == TriggerMode[SyncIn1]) {
    mode = SyncIn1;
  } else if (mode_str == TriggerMode[SyncIn2]) {
    mode = SyncIn2;
  } else if (mode_str == TriggerMode[SyncIn3]) {
    mode = SyncIn3;
  } else if (mode_str == TriggerMode[SyncIn4]) {
    mode = SyncIn4;
  }
  return mode;
}

void AvtVimbaCamera::printAllCameraFeatures(const CameraPtr& camera) {
  VmbErrorType err;
  FeaturePtrVector features;

  // The static details of a feature
  std::string strName;          // The name of the feature
  std::string strDisplayName;   // The display name of the feature
  std::string strTooltip;       // A short description of the feature
  std::string strDescription;   // A long description of the feature
  std::string strCategory;      // A category to group features
  std::string strSFNCNamespace; // The Std Feature Naming Convention namespace
  std::string strUnit;          // The measurement unit of the value
  VmbFeatureDataType eType;     // The data type of the feature

  // The changeable value of a feature
  VmbInt64_t nValue;
  std::string strValue;

  std::stringstream strError;

  // Fetch all features of our cam
  err = camera->GetFeatures(features);
  if (VmbErrorSuccess == err) {
    // Query all static details as well as the value of
    // all fetched features and print them out.
    for (FeaturePtrVector::const_iterator iter = features.begin();
         features.end() != iter;
         ++iter) {
      err = (*iter)->GetName(strName);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Name. Error code: " << err << "]";
        strName.assign(strError.str());
      }

      err = (*iter)->GetDisplayName(strDisplayName);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Display Name. Error code: " << err
                 << "]";
        strDisplayName.assign(strError.str());
      }

      err = (*iter)->GetToolTip(strTooltip);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Tooltip. Error code: " << err
                 << "]";
        strTooltip.assign(strError.str());
      }

      err = (*iter)->GetDescription(strDescription);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Description. Error code: " << err
                 << "]";
        strDescription.assign(strError.str());
      }

      err = (*iter)->GetCategory(strCategory);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Category. Error code: " << err
                 << "]";
        strCategory.assign(strError.str());
      }

      err = (*iter)->GetSFNCNamespace(strSFNCNamespace);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature SNFC Namespace. Error code: " << err
                 << "]";
        strSFNCNamespace.assign(strError.str());
      }

      err = (*iter)->GetUnit(strUnit);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Unit. Error code: " << err << "]";
        strUnit.assign(strError.str());
      }

      std::cout << "/// Feature Name: " << strName << std::endl;
      std::cout << "/// Display Name: " << strDisplayName << std::endl;
      std::cout << "/// Tooltip: " << strTooltip << std::endl;
      std::cout << "/// Description: " << strDescription << std::endl;
      std::cout << "/// SNFC Namespace: " << strSFNCNamespace << std::endl;
      std::cout << "/// Unit: " << strUnit << std::endl;
      std::cout << "/// Value: ";

      err = (*iter)->GetDataType(eType);
      if (VmbErrorSuccess != err) {
        std::cout << "[Could not get feature Data Type. Error code: " << err
                  << "]" << std::endl;
      } else {
        switch (eType) {
        case VmbFeatureDataBool:
          bool bValue;
          err = (*iter)->GetValue(bValue);
          if (VmbErrorSuccess == err) {
            std::cout << bValue << " bool" << std::endl;
          }
          break;
        case VmbFeatureDataEnum:
          err = (*iter)->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            std::cout << strValue << " str Enum" << std::endl;
          }
          break;
        case VmbFeatureDataFloat:
          double fValue;
          err = (*iter)->GetValue(fValue);
          { std::cout << fValue << " float" << std::endl; }
          break;
        case VmbFeatureDataInt:
          err = (*iter)->GetValue(nValue);
          { std::cout << nValue << " int" << std::endl; }
          break;
        case VmbFeatureDataString:
          err = (*iter)->GetValue(strValue);
          { std::cout << strValue << " str" << std::endl; }
          break;
        case VmbFeatureDataCommand:
        default:
          std::cout << "[None]" << std::endl;
          break;
        }
        if (VmbErrorSuccess != err) {
          std::cout << "Could not get feature value. Error code: " << err
                    << std::endl;
        }
      }

      std::cout << std::endl;
    }
  } else {
    std::cout << "Could not get features. Error code: "
              << api_.errorCodeToMessage(err) << std::endl;
  }
}

/** Change the Trigger configuration */
void AvtVimbaCamera::updateAcquisitionConfig(Config& config) {
  bool changed = false;
  if (config.acquisition_mode != config_.acquisition_mode || on_init_) {
    changed = true;
    setFeatureValue("AcquisitionMode", config.acquisition_mode.c_str());
  }
  if (config.acquisition_rate != config_.acquisition_rate || on_init_) {
    changed = true;
    double acquisition_frame_rate_limit;
    getFeatureValue("AcquisitionFrameRateLimit", acquisition_frame_rate_limit);
    std::cout << "------------------------------------" << std::endl;
    std::cout << "AcquisitionFrameRateLimit  " << acquisition_frame_rate_limit
              << std::endl;
    if (acquisition_frame_rate_limit < config.acquisition_rate) {
      double rate = (double)floor(acquisition_frame_rate_limit);
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "Max frame rate allowed: "
                             << acquisition_frame_rate_limit << ". Setting "
                             << rate << "...");
      config.acquisition_rate = 150;
    }
    setFeatureValue("AcquisitionFrameRateAbs",
                    static_cast<float>(config.acquisition_rate));
  }
  if (config.trigger_mode != config_.trigger_mode || on_init_) {
    changed = true;
    setFeatureValue("TriggerMode", config.trigger_mode.c_str());
  }
  if (config.trigger_selector != config_.trigger_selector || on_init_) {
    changed = true;
    setFeatureValue("TriggerSelector", config.trigger_selector.c_str());
  }
  if (config.trigger_source != config_.trigger_source || on_init_) {
    changed = true;
    setFeatureValue("TriggerSource", config.trigger_source.c_str());
  }
  if (config.trigger_activation != config_.trigger_activation || on_init_) {
    changed = true;
    setFeatureValue("TriggerActivation", config.trigger_activation.c_str());
  }
  if (config.trigger_delay != config_.trigger_delay || on_init_) {
    changed = true;
    setFeatureValue("TriggerDelayAbs", config.trigger_delay);
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New Acquisition and Trigger config ("
            << config.frame_id_ << ") : "
            << "\n\tAcquisitionMode         : " << config.acquisition_mode
            << " was " << config_.acquisition_mode
            << "\n\tAcquisitionFrameRateAbs : " << config.acquisition_rate
            << " was " << config_.acquisition_rate
            << "\n\tTriggerMode             : " << config.trigger_mode
            << " was " << config_.trigger_mode
            << "\n\tTriggerSource           : " << config.trigger_source
            << " was " << config_.trigger_source
            << "\n\tTriggerSelector         : " << config.trigger_selector
            << " was " << config_.trigger_selector
            << "\n\tTriggerActivation       : " << config.trigger_activation
            << " was " << config_.trigger_activation
            << "\n\tTriggerDelayAbs         : " << config.trigger_delay
            << " was " << config_.trigger_delay);
  }
}

/* Update the Iris config */
void AvtVimbaCamera::updateIrisConfig(Config& config) {
  bool changed = false;
  if (config.iris_auto_target != config_.iris_auto_target || on_init_) {
    changed = true;
    setFeatureValue("IrisAutoTarget",
                    static_cast<float>(config.iris_auto_target));
  }
  if (config.iris_mode != config_.iris_mode || on_init_) {
    changed = true;
    setFeatureValue("IrisMode", config.iris_mode.c_str());
  }
  //  if (config.iris_video_level != config_.iris_video_level || on_init_) {
  //    changed = true;
  //    setFeatureValue("IrisVideoLevel",
  //                    static_cast<VmbInt64_t>(config.iris_video_level));
  //  }
  if (config.iris_video_level_max != config_.iris_video_level_max || on_init_) {
    changed = true;
    setFeatureValue("IrisVideoLevelMax",
                    static_cast<float>(config.iris_video_level_max));
  }
  if (config.iris_video_level_min != config_.iris_video_level_min || on_init_) {
    changed = true;
    setFeatureValue("IrisVideoLevelMin",
                    static_cast<VmbInt64_t>(config.iris_video_level_min));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New Iris config ("
            << config.frame_id_ << ") : "
            << "\n\tIrisAutoTarget    : " << config.iris_auto_target << " was "
            << config_.iris_auto_target
        //      << "\n\tIrisMode          : " << config.iris_mode << " was " <<
        //      config_.iris_mode
        //      << "\n\tIrisVideoLevel    : " << config.iris_video_level << "
        //      was " << config_.iris_video_level
        //      << "\n\tIrisVideoLevelMax : " << config.iris_video_level_max <<
        //      " was " << config_.iris_video_level_max
        //      << "\n\tIrisVideoLevelMin : " << config.iris_video_level_min <<
        //      " was " << config_.iris_video_level_min
    );
  }
}

/** Change the Exposure configuration */
void AvtVimbaCamera::updateExposureConfig(Config& config) {
  bool changed = false;
  if (config.exposure != config_.exposure || on_init_) {
    changed = true;
    setFeatureValue("ExposureTimeAbs", static_cast<float>(config.exposure));
  }
  if (config.exposure_auto != config_.exposure_auto || on_init_) {
    changed = true;
    setFeatureValue("ExposureAuto", config.exposure_auto.c_str());
  }
  if (config.exposure_auto_alg != config_.exposure_auto_alg || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoAlg", config.exposure_auto_alg.c_str());
  }
  if (config.exposure_auto_tol != config_.exposure_auto_tol || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.exposure_auto_tol));
  }
  if (config.exposure_auto_max != config_.exposure_auto_max || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoMax",
                    static_cast<VmbInt64_t>(config.exposure_auto_max));
  }
  if (config.exposure_auto_min != config_.exposure_auto_min || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoMin",
                    static_cast<VmbInt64_t>(config.exposure_auto_min));
  }
  if (config.exposure_auto_outliers != config_.exposure_auto_outliers ||
      on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoOutliers",
                    static_cast<VmbInt64_t>(config.exposure_auto_outliers));
  }
  if (config.exposure_auto_rate != config_.exposure_auto_rate || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoRate",
                    static_cast<VmbInt64_t>(config.exposure_auto_rate));
  }
  if (config.exposure_auto_target != config_.exposure_auto_target || on_init_) {
    changed = true;
    setFeatureValue("ExposureAutoTarget",
                    static_cast<VmbInt64_t>(config.exposure_auto_target));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New Exposure config ("
            << config.frame_id_ << ") : "
            << "\n\tExposureTimeAbs       : " << config.exposure << " was "
            << config_.exposure << "\n\tExposureAuto          : "
            << config.exposure_auto << " was " << config_.exposure_auto
            << "\n\tExposureAutoAdjustTol : " << config.exposure_auto_tol
            << " was " << config_.exposure_auto_tol
            << "\n\tExposureAutoMax       : " << config.exposure_auto_max
            << " was " << config_.exposure_auto_max
            << "\n\tExposureAutoMin       : " << config.exposure_auto_min
            << " was " << config_.exposure_auto_min
            << "\n\tExposureAutoOutliers  : " << config.exposure_auto_outliers
            << " was " << config_.exposure_auto_outliers
            << "\n\tExposureAutoRate      : " << config.exposure_auto_rate
            << " was " << config_.exposure_auto_rate
            << "\n\tExposureAutoTarget    : " << config.exposure_auto_target
            << " was " << config_.exposure_auto_target);
  }
}

/** Change the Gain configuration */
void AvtVimbaCamera::updateGainConfig(Config& config) {
  bool changed = false;
  if (config.gain != config_.gain || on_init_) {
    changed = true;
    setFeatureValue("Gain", static_cast<float>(config.gain));
  }
  if (config.gain_auto != config_.gain_auto || on_init_) {
    changed = true;
    setFeatureValue("GainAuto", config.gain_auto.c_str());
  }
  if (config.gain_auto_tol != config_.gain_auto_tol || on_init_) {
    changed = true;
    setFeatureValue("GainAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.gain_auto_tol));
  }
  if (config.gain_auto_max != config_.gain_auto_max || on_init_) {
    changed = true;
    setFeatureValue("GainAutoMax", static_cast<float>(config.gain_auto_max));
  }
  if (config.gain_auto_min != config_.gain_auto_min || on_init_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_min));
  }
  if (config.gain_auto_outliers != config_.gain_auto_outliers || on_init_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_outliers));
  }
  if (config.gain_auto_rate != config_.gain_auto_rate || on_init_) {
    changed = true;
    setFeatureValue("GainAutoOutliers",
                    static_cast<VmbInt64_t>(config.gain_auto_rate));
  }
  if (config.gain_auto_target != config_.gain_auto_target || on_init_) {
    changed = true;
    setFeatureValue("GainAutoRate",
                    static_cast<VmbInt64_t>(config.gain_auto_target));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New Gain config ("
            << config.frame_id_ << ") : "
            << "\n\tGain              : " << config.gain << " was "
            << config_.gain << "\n\tGainAuto          : " << config.gain_auto
            << " was " << config_.gain_auto << "\n\tGainAutoAdjustTol : "
            << config.gain_auto_tol << " was " << config_.gain_auto_tol
            << "\n\tGainAutoMax       : " << config.gain_auto_max << " was "
            << config_.gain_auto_max << "\n\tGainAutoMin       : "
            << config.gain_auto_min << " was " << config_.gain_auto_min
            << "\n\tGainAutoOutliers  : " << config.gain_auto_outliers
            << " was " << config_.gain_auto_outliers
            << "\n\tGainAutoRate      : " << config.gain_auto_rate << " was "
            << config_.gain_auto_rate << "\n\tGainAutoTarget    : "
            << config.gain_auto_target << " was " << config_.gain_auto_target);
  }
}

/** Change the White Balance configuration */
void AvtVimbaCamera::updateWhiteBalanceConfig(Config& config) {
  bool changed = false;
  if (config.balance_ratio_abs != config_.balance_ratio_abs || on_init_) {
    changed = true;
    setFeatureValue("BalanceRatioAbs",
                    static_cast<float>(config.balance_ratio_abs));
  }
  if (config.balance_ratio_selector != config_.balance_ratio_selector ||
      on_init_) {
    changed = true;
    setFeatureValue("BalanceRatioSelector",
                    config.balance_ratio_selector.c_str());
  }
  if (config.whitebalance_auto != config_.whitebalance_auto || on_init_) {
    changed = true;
    setFeatureValue("BalanceWhiteAuto", config.whitebalance_auto.c_str());
  }
  if (config.whitebalance_auto_tol != config_.whitebalance_auto_tol ||
      on_init_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.whitebalance_auto_tol));
  }
  if (config.whitebalance_auto_rate != config_.whitebalance_auto_rate ||
      on_init_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoRate",
                    static_cast<VmbInt64_t>(config.whitebalance_auto_rate));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New White Balance config ("
            << config.frame_id_ << ") : "
            << "\n\tBalanceRatioAbs           : " << config.balance_ratio_abs
            << " was " << config_.balance_ratio_abs
            << "\n\tBalanceRatioSelector      : "
            << config.balance_ratio_selector << " was "
            << config_.balance_ratio_selector
            << "\n\tBalanceWhiteAuto          : " << config.whitebalance_auto
            << " was " << config_.whitebalance_auto
            << "\n\tBalanceWhiteAutoAdjustTol : "
            << config.whitebalance_auto_tol << " was "
            << config_.whitebalance_auto_tol
            << "\n\tBalanceWhiteAutoRate      : "
            << config.whitebalance_auto_rate << " was "
            << config_.whitebalance_auto_rate);
  }
}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updatePtpModeConfig(Config& config) {
  bool changed = false;
  if (config.ptp_mode != config_.ptp_mode || on_init_) {
    changed = true;
    setFeatureValue("PtpMode", config.ptp_mode.c_str());
  }

  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "New PTP config (" << config.frame_id_ << ") : "
                                          << "\n\tPtpMode                   : "
                                          << config.ptp_mode << " was "
                                          << config_.ptp_mode);
  }
}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updateImageModeConfig(Config& config) {
  bool changed = false;
  if (config.decimation_x != config_.decimation_x || on_init_) {
    changed = true;
    setFeatureValue("DecimationHorizontal",
                    static_cast<VmbInt64_t>(config.decimation_x));
  }
  if (config.decimation_y != config_.decimation_y || on_init_) {
    changed = true;
    setFeatureValue("DecimationVertical",
                    static_cast<VmbInt64_t>(config.decimation_y));
  }
  if (config.binning_x != config_.binning_x || on_init_) {
    changed = true;
    setFeatureValue("BinningHorizontal",
                    static_cast<VmbInt64_t>(config.binning_x));
  }
  if (config.binning_y != config_.binning_y || on_init_) {
    changed = true;
    setFeatureValue("BinningVertical",
                    static_cast<VmbInt64_t>(config.binning_y));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New Image Mode config ("
            << config.frame_id_ << ") : "
            << "\n\tDecimationHorizontal : " << config.decimation_x << " was "
            << config_.decimation_x << "\n\tDecimationVertical   : "
            << config.decimation_y << " was " << config_.decimation_y
            << "\n\tBinningHorizontal    : " << config.binning_x << " was "
            << config_.binning_x << "\n\tBinningVertical      : "
            << config.binning_y << " was " << config_.binning_y);
  }
}

/** Change the ROI configuration */
void AvtVimbaCamera::updateROIConfig(Config& config) {
  bool changed = false;

  // Region of interest configuration
  // Make sure ROI fits in image

  int max_width, max_height;
  getFeatureValue("WidthMax", max_width);
  getFeatureValue("HeightMax", max_height);

  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  max_width *= binning_or_decimation_x;
  max_height *= binning_or_decimation_y;

  config.width = std::min(config.width, (int)max_width);
  config.height = std::min(config.height, (int)max_height);
  config.roi_offset_x = std::min(config.roi_offset_x, config.width - 1);
  config.roi_offset_y = std::min(config.roi_offset_y, config.height - 1);
  config.roi_width =
      std::min(config.roi_width, config.width - config.roi_offset_x);
  config.roi_height =
      std::min(config.roi_height, config.height - config.roi_offset_y);

  // If width or height is 0, set it as large as possible
  int width =
      config.roi_width ? config.roi_width : max_width - config.roi_offset_x;
  int height =
      config.roi_height ? config.roi_height : max_height - config.roi_offset_y;

  // Adjust full-res ROI to binning ROI
  /// @todo Replicating logic from polledCallback
  int offset_x = config.roi_offset_x;
  int offset_y = config.roi_offset_y;
  unsigned int right_x = (offset_x + width + binning_or_decimation_x - 1);
  unsigned int bottom_y = (offset_y + height + binning_or_decimation_y - 1);

  // Rounding up is bad when at max resolution which is not divisible by the
  // amount of binning
  right_x = std::min(right_x, (unsigned)(config.width));
  bottom_y = std::min(bottom_y, (unsigned)(config.height));
  width = right_x - offset_x;
  height = bottom_y - offset_y;

  config.width = width / binning_or_decimation_x;
  config.height = height / binning_or_decimation_y;

  config.roi_offset_x = offset_x / binning_or_decimation_x;
  config.roi_offset_y = offset_y / binning_or_decimation_y;

  if (config.roi_offset_x != config_.roi_offset_x || on_init_) {
    changed = true;
    setFeatureValue("OffsetX", static_cast<VmbInt64_t>(config.roi_offset_x));
  }
  if (config.roi_offset_y != config_.roi_offset_y || on_init_) {
    changed = true;
    setFeatureValue("OffsetY", static_cast<VmbInt64_t>(config.roi_offset_y));
  }
  if (config.width != config_.width || on_init_) {
    changed = true;
    setFeatureValue("Width", static_cast<VmbInt64_t>(config.width));
  }
  if (config.height != config_.height || on_init_) {
    changed = true;
    setFeatureValue("Height", static_cast<VmbInt64_t>(config.height));
  }

  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "New ROI config ("
                           << config.frame_id_ << ") : "
                           << "\n\tOffsetX : " << config.roi_offset_x << " was "
                           << config_.roi_offset_x
                           << "\n\tOffsetY : " << config.roi_offset_y << " was "
                           << config_.roi_offset_y
                           << "\n\tWidth   : " << config.width << " was "
                           << config_.width << "\n\tHeight  : " << config.height
                           << " was " << config_.height);
  }
}

/** Change the Bandwidth configuration */
void AvtVimbaCamera::updateBandwidthConfig(Config& config) {
  bool changed = false;
  if (config.stream_bytes_per_second != config_.stream_bytes_per_second ||
      on_init_) {
    changed = true;
    setFeatureValue("StreamBytesPerSecond",
                    static_cast<VmbInt64_t>(config.stream_bytes_per_second));
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "New Bandwidth config ("
                           << config.frame_id_ << ") : "
                           << "\n\tStreamBytesPerSecond : "
                           << config.stream_bytes_per_second << " was "
                           << config_.stream_bytes_per_second);
  }
}

/** Change the Pixel Format configuration */
void AvtVimbaCamera::updatePixelFormatConfig(Config& config) {
  bool changed = false;
  if (config.pixel_format != config_.pixel_format || on_init_) {
    changed = true;
    setFeatureValue("PixelFormat", config.pixel_format.c_str());
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "New PixelFormat config ("
                           << config.frame_id_ << ") : "
                           << "\n\tPixelFormat : " << config.pixel_format
                           << " was " << config_.pixel_format);
  }
}

/** Change the GPIO configuration */
void AvtVimbaCamera::updateGPIOConfig(Config& config) {
  bool changed = false;
  if (config.sync_in_selector != config_.sync_in_selector || on_init_) {
    changed = true;
    setFeatureValue("SyncInSelector", config.sync_in_selector.c_str());
  }
  if (config.sync_out_polarity != config_.sync_out_polarity || on_init_) {
    changed = true;
    setFeatureValue("SyncOutPolarity", config.sync_out_polarity.c_str());
  }
  if (config.sync_out_selector != config_.sync_out_selector || on_init_) {
    changed = true;
    setFeatureValue("SyncOutSelector", config.sync_out_selector.c_str());
  }
  if (config.sync_out_source != config_.sync_out_source || on_init_) {
    changed = true;
    setFeatureValue("SyncOutSource", config.sync_out_source.c_str());
  }
  if (changed && show_debug_prints_) {
    RCLCPP_INFO_STREAM(
        node_handle_->get_logger(),
        "New GPIO config ("
            << config.frame_id_ << ") : "
            << "\n\tSyncInSelector  : " << config.sync_in_selector << " was "
            << config_.sync_in_selector << "\n\tSyncOutPolarity : "
            << config.sync_out_polarity << " was " << config_.sync_out_polarity
            << "\n\tSyncOutSelector : " << config.sync_out_selector << " was "
            << config_.sync_out_selector << "\n\tSyncOutSource   : "
            << config.sync_out_source << " was " << config_.sync_out_source);
  }
}

bool AvtVimbaCamera::ptpSyncTrigger(void) {
  // wait until the camera is PTP synchronized
  // TODO: this polling approach should be replaced by an event callback
  diagnostic_msg_ = "Synchronizing trigger with PTP";
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "Synchronizing trigger with PTP");
  int syncTimeout = 10; // wait at most for 10 seconds
  for (int i = 0; i < syncTimeout; i++) {
    std::string ptpStatus;
    getFeatureValue("PtpStatus", ptpStatus);
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       "ptpStatus: " << ptpStatus << " (attempt " << i + 1
                                     << " of " << syncTimeout << ")");
    if (ptpStatus == "Disabled") {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "Cannot sync trigger when ptpMode is \"Off\"");
      return false;
    } else if (ptpStatus == "Listening" || ptpStatus == "Uncalibrated") {
      // Not yet sync'ed.  Wait for 1 second and try again.
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else if (ptpStatus == "Slave" || ptpStatus == "Master" ||
               ptpStatus == "Passive") {
      break; // sync'ed
    } else {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                         "Unknown ptpStatus -- not PTP syncronized");
      return false;
    }
  }

  // Determine a future moment at which to trigger
  // This should be an integer number of trigger periods
  VmbInt64_t ptpAcquisitionGateTime;
  if (runCommand("GevTimestampControlLatch")) {
    VmbInt64_t ticks, tickFreq;
    float acquisitionRate;
    getFeatureValue("GevTimestampValue", ticks);
    getFeatureValue("GevTimestampTickFrequency",
                    tickFreq); // ticks per second (typically: 1e9)
    getFeatureValue("AcquisitionFrameRateAbs",
                    acquisitionRate); // frames per second

    // Determine at which tick to trigger the camera
    double acquisitionPeriodTicks =
        nearbyint(1.0 / ((double)acquisitionRate) *
                  ((double)tickFreq)); // ticks per period
    double numPeriods = nearbyint(
        ((double)ticks) /
        acquisitionPeriodTicks); // number of whole periods since ticks=0
    // Add one second (i.e., tickFreq ticks) to allow commands to propagate
    ptpAcquisitionGateTime =
        static_cast<VmbInt64_t>(numPeriods * acquisitionPeriodTicks) + tickFreq;
    RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                       std::endl
                           << "Current camera ticks = " << ticks << std::endl
                           << "tickFreq = " << tickFreq << std::endl
                           << "acquisitionRate = " << acquisitionRate
                           << std::endl
                           << "acquisitionPeriodTicks = "
                           << acquisitionPeriodTicks << std::endl
                           << "numPeriods = " << numPeriods << std::endl
                           << "ptpAcquisitionGateTime = "
                           << ptpAcquisitionGateTime);

  } else {
    RCLCPP_WARN_STREAM(
        node_handle_->get_logger(),
        "GevTimestampControlLatch failed -- not PTP syncronized");
    return false;
  }

  // set the next trigger instant
  return setFeatureValue("PtpAcquisitionGateTime", ptpAcquisitionGateTime);
}

#if 0 // ros1
void AvtVimbaCamera::getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Serial", guid_);
  stat.add("Info", diagnostic_msg_);
  // stat.add("Intrinsics", intrinsics_);
  // stat.add("Total frames dropped", frames_dropped_total_);
  // stat.add("Total frames", frames_completed_total_);

  // if (frames_completed_total_ > 0) {
  //   stat.add("Total % frames dropped", 100.*(double)frames_dropped_total_/frames_completed_total_);
  // }

  // if (frames_completed_acc_.sum() > 0) {
  //   stat.add("Recent % frames dropped", 100.*frames_dropped_acc_.sum()/frames_completed_acc_.sum());
  // }

  switch (camera_state_) {
    case OPENING:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Opening camera");
      break;
    case IDLE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera is idle");
      break;
    case OK:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera is streaming");
      break;
    case CAMERA_NOT_FOUND:
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Cannot find requested camera %s", guid_.c_str());
      // stat.add("Available Cameras", getAvailableCameras());
      break;
    case FORMAT_ERROR:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Problem retrieving frame");
      break;
    case ERROR:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera has encountered an error");
      break;
    default:
      break;
  }
}

#endif // ros1
} // namespace avt_vimba_camera
