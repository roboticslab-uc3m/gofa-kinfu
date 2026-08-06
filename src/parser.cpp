#include "parser.h"
#include <yaml-cpp/yaml.h>

void parseDHParameters(const std::string& filename, std::vector<DHParameters>& dh_params) {
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& param : config["dh_parameters"]) {
        DHParameters dh;
        dh.a = param["a"].as<double>();
        dh.alpha = param["alpha"].as<double>();
        dh.d = param["d"].as<double>();
        dh.theta = param["theta"].as<double>();
        dh_params.push_back(dh);
    }
}

void parseJointLimits(const std::string& filename, std::vector<JointLimits>& joint_limits) {
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& node : config["joint_limits"]) {
        JointLimits limits;
        limits.min = node["min"].as<double>();
        limits.max = node["max"].as<double>();
        joint_limits.push_back(limits);
    }
}

void parseInputParameters(const std::string& filename, InputParameters& input_params) {
    YAML::Node config = YAML::LoadFile(filename);
    input_params.centroid = config["input_parameters"]["centroid"].as<std::vector<double>>();
    input_params.radius = config["input_parameters"]["radius"].as<double>();
    input_params.height = config["input_parameters"]["height"].as<double>();
    input_params.inclination = config["input_parameters"]["inclination"].as<double>();
    input_params.resolution = config["input_parameters"]["resolution"].as<int>();
    input_params.speed = config["input_parameters"]["speed"].as<double>();
    input_params.start_position = config["input_parameters"]["start_position"].as<double>();
    input_params.end_position = config["input_parameters"]["end_position"].as<double>();

    // Leer las posiciones articulares iniciales
    input_params.initial_joint_positions = config["initial_joint_positions"].as<std::vector<double>>();
}
