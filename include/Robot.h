#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <rl_cartesian_control_msgs/action/joint_trajectory.hpp>

#include <boost/asio/serial_port.hpp>

#include "TrajectoryGenerator.h"

struct DHParameters
{
    double a;
    double alpha;
    double d;
    double theta;
};

struct JointLimits
{
    double min;
    double max;
};

class Robot
{
public:
    enum joint_flag {REACHABLE, UNREACHABLE};

    ~Robot();

    bool handleExecute(double speed, double radius, const std::vector<double> & initialJointPositions);
    bool doSimultaneous(boost::asio::serial_port & serial, int angle, int turningTime);
    bool doSequential(boost::asio::serial_port & serial, int angle, int turningTime);

    bool handleSolveIK(const std::vector<Point> & points, const TrajectoryGenerator & trajectory, bool debug);
    bool handlePlotJoints(const TrajectoryGenerator & trajectory, bool debug, const std::string & filename = "");

    static std::unique_ptr<Robot> parseRobotConfiguration(const std::string & filename);

private:
    Robot();
    Robot(const Robot &) = delete;
    Robot & operator=(const Robot &) = delete;

    void parseDHParameters(const std::string & filename);
    void parseJointLimits(const std::string & filename);

    bool sendJointTrajectoryGoal(const rl_cartesian_control_msgs::action::JointTrajectory::Goal & goalMsg);

    std::vector<DHParameters> dhParams;
    std::vector<JointLimits> jointLimits;
    std::vector<std_msgs::msg::Float32MultiArray> jointTrajectory;
    std::vector<joint_flag> jointFlags;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr jointPublisher;
    rclcpp_action::Client<rl_cartesian_control_msgs::action::JointTrajectory>::SharedPtr jointTrajectoryAction;
};

#endif // __ROBOT_H__
