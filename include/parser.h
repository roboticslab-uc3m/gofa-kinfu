#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
};

struct JointLimits {
    double min;
    double max;
};

struct InputParameters {
    std::vector<double> centroid;
    double radius;
    double height;
    double inclination;
    int resolution;
    double speed;
    double start_position; // �ngulo inicial en grados
    double end_position; // �ngulo final en grados
    std::vector<double> initial_joint_positions;
};

void parseDHParameters(const std::string& filename, std::vector<DHParameters>& dh_params);
void parseJointLimits(const std::string& filename, std::vector<JointLimits>& joint_limits);
void parseInputParameters(const std::string& filename, InputParameters& input_params);

#endif // PARSER_H
