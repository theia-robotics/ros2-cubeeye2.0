#include <sstream>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ModelParameter.h"
#include "ICubeModelParameter.h"
#include "SCubeModelParameter.h"

std::shared_ptr<ModelParameter> ModelParameter::create(meere::sensor::sptr_camera camera) {
    if (camera->source()->name() == "S100D"
        || camera->source()->name() == "S110D"
        || camera->source()->name() == "S111D"
        || camera->source()->name() == "E100") {
        return std::make_shared<SCubeModelParameter>(camera);
    }
    else if (camera->source()->name() == "I200D") {
        return std::make_shared<ICubeModelParameter>(camera);
    }

    return nullptr;
}

void ModelParameter::build(ModelParameter::Descriptor descriptors[], size_t len) {
    for (size_t i = 0; i < len; i++) {
        mDescriptors.push_back(descriptors[i]);
    }
}

void ModelParameter::addTo(rclcpp::Node* node) {
    // declare parameters
    for (const auto & _desc : mDescriptors) {
        std::string _name = _desc.name;
        
        if (_desc.param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
            rcl_interfaces::msg::IntegerRange _range;
            _range.from_value = _desc.from_value;
            _range.to_value = _desc.to_value;
            _range.step = _desc.step;

            rcl_interfaces::msg::ParameterDescriptor _descriptor;
            _descriptor.name = _desc.name;
            _descriptor.type = _desc.param_type;
            _descriptor.description = _desc.description;
            _descriptor.integer_range.push_back(_range);
            _descriptor.dynamic_typing = true;

            node->declare_parameter(_name, _desc.default_value, _descriptor);
        }
        else if (_desc.param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
            rcl_interfaces::msg::ParameterDescriptor _descriptor;
            _descriptor.name = _desc.name;
            _descriptor.type = _desc.param_type;
            _descriptor.description = _desc.description;
            _descriptor.dynamic_typing = true;
            
            node->declare_parameter(_name, _desc.default_value != 0, _descriptor);
        }
    }

    mCallbackHandle = node->add_on_set_parameters_callback(std::bind(&ModelParameter::parametersCallback, this, std::placeholders::_1));    
}

rcl_interfaces::msg::SetParametersResult ModelParameter::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult _result;
    _result.successful = true;
    _result.reason = "success";
    for (const auto &_parameter : parameters)
    {
        auto it = std::find_if(mDescriptors.begin(), mDescriptors.end(), 
                                        [_parameter](const ModelParameter::Descriptor & desc) -> bool { return desc.name == _parameter.get_name(); });

        if (it != mDescriptors.end()) {
            meere::sensor::sptr_property _prop = nullptr;
            
            switch ((*it).data_type) {
            case meere::sensor::DataType::Boolean:
                _prop = meere::sensor::make_property_bool((*it).name, _parameter.as_bool());
                break;
            case meere::sensor::DataType::U16:
                _prop = meere::sensor::make_property_16u((*it).name, _parameter.as_int());
                break;
            case meere::sensor::DataType::U8:
                _prop = meere::sensor::make_property_8u((*it).name, _parameter.as_int());
                break; 
            case meere::sensor::DataType::S16:
                _prop = meere::sensor::make_property_16s((*it).name, _parameter.as_int());
                break;
            case meere::sensor::DataType::S8:
                _prop = meere::sensor::make_property_8s((*it).name, _parameter.as_int());
                break;
            }

            if (_prop != nullptr) {
                if (meere::sensor::result::success != mCamera->setProperty(_prop)) {
                    _result.successful = false;
                    _result.reason = "setProperty(" + _prop->key() + ") is failed.";
                    break;
                }
            }
        }
        else {
            _result.successful = false;
            _result.reason = "parameter (" + _parameter.get_name() + ") is not defined.";
            break;
        }
    }
    return _result;
}

void ModelParameter::removeFrom(rclcpp::Node* node) {
    for (const auto & _desc : mDescriptors) {
        std::string _name = _desc.name;
        node->undeclare_parameter(_name);
    }

    node->remove_on_set_parameters_callback(mCallbackHandle.get());
}
