#include "bvs_localization/sources/source_factory.h"
#include "bvs_localization/sources/novatel_oem7_inspva.h"
#include "bvs_localization/sources/novatel_oem7_bestpos.h"
#include "bvs_localization/sources/novatel_oem7_bestvel.h"
#include "bvs_localization/sources/rear_wheel_velocity.h"
#include "bvs_localization/sources/front_wheel_velocity.h"

#include <iostream>

namespace bvs_localization {
namespace sources {

SourceFactory::SourceFactory(Resources resources):
    resources_(resources),
    config_(Config::Unknown)
{}

std::map<std::string, estimate::EstimateSource::SharedPtr>
SourceFactory::generate(std::string source_config) {
    if(source_config == "param") {
        config_ = Config::Param;
    } else {
        config_ = Config::Yaml;
    }

    auto sources = getSourceList();

    std::map<std::string, estimate::EstimateSource::SharedPtr> result;

    for(auto it = sources.begin(); it != sources.end(); ++it) {
        try {
            result[*it] = generateSource(*it);
        } catch(std::exception& err) {
            std::cerr << "Unable to add source `" + *it << "`: " << err.what() << std::endl;
        }
    }

    return result;
}

estimate::EstimateSource::SharedPtr
SourceFactory::generateSource(
    std::string source_name
) {
    std::string source_type = getParamString(source_name, "type");
    std::cerr << "Loading source `" + source_name + "` of type `" + source_type + "`" << std::endl;
    if(source_type == "novatel_oem7_inspva") {
        return std::make_shared<NovatelOem7INSPVA>(
            resources_.node,
            source_name,
            getParamString(source_name, "topic"),
            getParamDouble(source_name, "health_lim_age"),
            resources_.converter
        );
    }
    if(source_type == "rear_wheel_velocity") {
        return std::make_shared<RearWheelVelocity>(
            resources_.node,
            source_name,
            getParamString(source_name, "topic")
        );
    }
    if(source_type == "front_wheel_velocity") {
        return std::make_shared<FrontWheelVelocity>(
            resources_.node,
            source_name,
            getParamString(source_name, "topic")
        );
    }
    if(source_type == "novatel_oem7_bestpos") {
        return std::make_shared<NovatelOem7BESTPOS>(
            resources_.node,
            source_name,
            getParamString(source_name, "topic_bestpos"),
            getParamDouble(source_name, "health_lim_stddev"),
            getParamDouble(source_name, "health_lim_age"),
            resources_.converter
        );
    }
    if(source_type == "novatel_oem7_bestvel") {
        return std::make_shared<NovatelOem7BESTVEL>(
            resources_.node,
            source_name,
            getParamString(source_name, "topic_bestvel"),
            getParamString(source_name, "topic_inspva"),
            getParamBool(source_name, "orient_to_inspva"),
            getParamDouble(source_name, "health_lim_age"),
            resources_.converter
        );
    }
    throw std::runtime_error("Source `" + source_name + "` has type `" + source_type + "` that is unknown");
}


std::string
SourceFactory::getParamString(std::string source_name, std::string param_name) {
    if(config_ == Config::Param) {
        resources_.node->declare_parameter("sources." + source_name + "." + param_name);
        return resources_.node->get_parameter("sources." + source_name + "." + param_name).as_string();
    }
    if(config_ == Config::Yaml) {
        throw std::runtime_error("Yaml source is not yet implemented");
    }
    throw std::runtime_error("Unable to load paramter [" + param_name + "] for source [" + source_name + "]");
}

double
SourceFactory::getParamDouble(std::string source_name, std::string param_name) {
    if(config_ == Config::Param) {
        resources_.node->declare_parameter("sources." + source_name + "." + param_name);
        return resources_.node->get_parameter("sources." + source_name + "." + param_name).as_double();
    }
    if(config_ == Config::Yaml) {
        throw std::runtime_error("Yaml source is not yet implemented");
    }
    throw std::runtime_error("Unable to load paramter [" + param_name + "] for source [" + source_name + "]");
}

bool
SourceFactory::getParamBool(std::string source_name, std::string param_name) {
    if(config_ == Config::Param) {
        resources_.node->declare_parameter("sources." + source_name + "." + param_name);
        return resources_.node->get_parameter("sources." + source_name + "." + param_name).as_bool();
    }
    if(config_ == Config::Yaml) {
        throw std::runtime_error("Yaml source is not yet implemented");
    }
    throw std::runtime_error("Unable to load paramter [" + param_name + "] for source [" + source_name + "]");
}

std::vector<std::string>
SourceFactory::getSourceList() {
    if(config_ == Config::Param) {
        resources_.node->declare_parameter("sources", std::vector<std::string>({}));
        return resources_.node->get_parameter("sources").as_string_array();
    }
    if(config_ == Config::Yaml) {
        throw std::runtime_error("Yaml source is not yet implemented");
    }
    throw std::runtime_error("Unable to load source list");
}

std::vector<std::string>
SourceFactory::getSourcePriority() {
    if(config_ == Config::Param) {
        resources_.node->declare_parameter("source_priority", std::vector<std::string>({}));
        return resources_.node->get_parameter("source_priority").as_string_array();
    }
    if(config_ == Config::Yaml) {
        throw std::runtime_error("Yaml source is not yet implemented");
    }
    throw std::runtime_error("Unable to load source list");
}

} /* namespace sources */
} /* namespace bvs_localization */
