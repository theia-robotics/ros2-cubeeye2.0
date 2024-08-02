#include <sstream>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "SCubeModelParameter.h"

static ModelParameter::Descriptor descriptors[] = {
    { "amplitude_threshold", "amplitude threshold", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U16, 0, 4095, 1, 5 },
    { "scattering_threshold", "scattering threshold", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U16, 0, 4095, 1, 100 },
    { "auto_exposure", "auto exposure on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1 },
    { "integration_time", "integration time (depends on auto_exposure)", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U8, 1, 61, 1, 54 },
    { "standby_mode", "standby mode",rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U16, 0, 1, 1, 1 },
    { "illumination", "illumination on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1},
    { "flying_pixel_remove_filter", "flying pixel removal filter on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1},
    { "noise_filter1", "noise reduction filter1 on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1},
    { "noise_filter2", "noise reduction filter2 on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1},
    { "noise_filter3", "noise reduction filter3 on/off", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 1},
    { "corner_mask", "invalidation by illumination mask on/off (only support for S110D)", rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, meere::sensor::DataType::Boolean, 0, 0, 0, 0},
    { "depth_offset", "depth offset", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::S16, -4095, 4095, 1, 0 },
    { "depth_range_min", "depth range min", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U16, 0, 65535, 1, 150 },
    { "depth_range_max", "depth range max", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U16, 0, 65535, 1, 6000 },
    { "depth_error_remove_threshold", "depth error remove threshold", rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, meere::sensor::DataType::U8, 0, 255, 1, 255 },
};

SCubeModelParameter::SCubeModelParameter(meere::sensor::sptr_camera camera) : ModelParameter(camera) {
    build(descriptors, sizeof(descriptors) / sizeof(ModelParameter::Descriptor));
}
