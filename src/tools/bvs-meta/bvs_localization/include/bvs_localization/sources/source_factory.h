/**
 * @brief source factory can generate estimation sources from a variety of configs
 **/
#ifndef BVS_LOCALIZATION_SOURCES_SOURCE_FACTORY_H_
#define BVS_LOCALIZATION_SOURCES_SOURCE_FACTORY_H_

// Application
#include "bvs_localization/estimate/estimate_source.h"
#include "bvs_utils/geodetic_conv.h"

// ROS
#include "rclcpp/rclcpp.hpp"

namespace bvs_localization {
namespace sources {

class SourceFactory {
public:
    //! Resources that different sources might need
    //!     in order to process properly / generate
    struct Resources {
        rclcpp::Node::SharedPtr node;
        bvs_utils::GeodeticConverter::SharedPtr converter;
    };

    //! Used to set what the config source is (yaml or ros param)
    enum class Config {
        Unknown,
        Yaml,
        Param
    };

    /**
     * @brief resources stay alive the same lifecycle as the Factory
     * @param resources are used to generate any sources
     **/
    SourceFactory(Resources resources);

    /**
     * @brief used to generate all the sources from a config
     * @param config_source either 'param' or a path to a yaml file
     * @return a map of estimation sources
     **/
    std::map<std::string, estimate::EstimateSource::SharedPtr>
    generate(std::string config_source);

    /**
     * @brief generates a single odometry source of a given type / name
     * @param source_name the name of the source to generate (taken from config)
     **/
    estimate::EstimateSource::SharedPtr
    generateSource(std::string source_name);

    /**
     * @brief abstracts away the config to a source name and param name
     * @param source_name sets basically an additional namespace for the param
     * @param param_name the name of the parameter value to get
     **/
    std::string getParamString(std::string source_name, std::string param_name);
    double getParamDouble(std::string source_name, std::string param_name);
    bool getParamBool(std::string source_name, std::string param_name);

    /**
     * @brief gets the source list from whatever config is chosen
     * @return a list of config sources
     **/
    std::vector<std::string>
    getSourceList();

    /**
     * @brief gets the source priority list
     * @return a list of config sources from the selected config
     **/
    std::vector<std::string>
    getSourcePriority();

private:
    Resources resources_;
    Config config_;

};


} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_SOURCES_SOURCE_FACTORY_H_ */