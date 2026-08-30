#include "Robot.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <yaml-cpp/yaml.h>
#include <matplotlibcpp.h>

#include <boost/asio.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>

#include <SceneReconstructionIDL.h>

namespace plt = matplotlibcpp;

Robot::Robot()
{
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("gofa_kinfu");
    jointPublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("/command/joint", 10);
    jointTrajectoryAction = rclcpp_action::create_client<rl_cartesian_control_msgs::action::JointTrajectory>(node, "/trajectory/joint");
}

Robot::~Robot()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void Robot::parseDHParameters(const std::string & filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    for (const auto & param : config["dh_parameters"])
    {
        DHParameters dh;
        dh.a = param["a"].as<double>();
        dh.alpha = param["alpha"].as<double>();
        dh.d = param["d"].as<double>();
        dh.theta = param["theta"].as<double>();

        dhParams.push_back(dh);
    }
}

void Robot::parseJointLimits(const std::string & filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    for (const auto & node : config["joint_limits"])
    {
        JointLimits limits;
        limits.min = node["min"].as<double>();
        limits.max = node["max"].as<double>();

        jointLimits.push_back(limits);
    }
}

std::unique_ptr<Robot> Robot::parseRobotConfiguration(const std::string & filename)
{
    // can't use std::make_unique here due to the private ctor
    std::unique_ptr<Robot> robot(new Robot());

    try
    {
        robot->parseDHParameters(filename);
        robot->parseJointLimits(filename);
        return robot;
    }
    catch (const YAML::Exception & e)
    {
        std::cerr << "Error parsing robot configuration file: " << e.what() << std::endl;
        return nullptr;
    }
}

bool Robot::doSimultaneous(boost::asio::serial_port & serial, int angle, int turningTime)
{
    std::cout << "--- EXECUTING SIMULTANEOUSLY ---\n";

    std::string msg = "G" + std::to_string(angle) + "T" + std::to_string(turningTime) + "\n";

    if (serial.is_open())
    {
        std::cout << "Triggering rotating platform: " << msg;
        boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));
    }

    const auto delay = turningTime / jointTrajectory.size();

    for (const auto & jointMsg : jointTrajectory)
    {
        jointPublisher->publish(jointMsg);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

bool Robot::doSequential(boost::asio::serial_port & serial, int angle, int turningTime)
{
    std::cout << "--- EXECUTING SEQUENTIALLY ---\n";

    const auto delay = turningTime / jointTrajectory.size();

    for (const auto & jointMsg : jointTrajectory)
    {
        jointPublisher->publish(jointMsg);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }

    std::cout << "The GoFa robot has reached the end point of the trajectory\n";

    if (serial.is_open())
    {
        std::string msg = "G" + std::to_string(angle) + "T" + std::to_string(turningTime) + "\n";
        std::cout << "Triggering platform sequentially: " << msg;
        boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));

        // Synchronize the C++ thread with the rotation time of the Arduino table
        std::cout << "Waiting for the platform to finish rotating (" << turningTime << " ms)...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(turningTime));
        std::cout << "Platform rotation completed\n";
    }

    return true;
}

bool Robot::sendJointTrajectoryGoal(const rl_cartesian_control_msgs::action::JointTrajectory::Goal & goalMsg)
{
    auto goalHandleFuture = jointTrajectoryAction->async_send_goal(goalMsg);

    if (rclcpp::spin_until_future_complete(node, goalHandleFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "Error: failed to send goal\n";
        return false;
    }

    auto goalHandle = goalHandleFuture.get();

    if (!goalHandle)
    {
        std::cerr << "Error: failed to get goal handle\n";
        return false;
    }

    auto result = jointTrajectoryAction->async_get_result(goalHandle);

    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "Error: failed to receive result\n";
        return false;
    }

    return true;
}

bool Robot::handleExecute(double speed, double radius, const std::vector<double> & initialJointPositions)
{
    int angle = 0;
    int turningTime = 0;
    int mode180 = 1; // 1: simultaneous, 2: sequential

    std::cout << "Enter the platform angle (180 or 360): ";
    std::cin >> angle;
    std::cout << "The robot speed is: " << speed << '\n';

    if (speed <= 0)
    {
        std::cout << "Error: speed must be greater than 0, setting default to 20 mm/s\n";
        speed = 20.0;
    }

    float distance = std::abs(angle) * radius * KDL::deg2rad;
    turningTime = static_cast<int>((distance / speed) * 1000.0);

    std::cout << "Estimated distance for the robot: " << distance << " mm\n";
    std::cout << "Calculated time for the platform: " << turningTime << " ms\n";

    if (std::abs(angle) == 180)
    {
        std::cout << "How do you want to perform the 180-degree scan?\n";
        std::cout << " [1] Simultaneously (robot and platform at the same time)\n";
        std::cout << " [2] Sequentially (first the robot completes its trajectory, then the platform rotates)\n";
        std::cout << "Select an option: ";
        std::cin >> mode180;
    }

    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "\nPress Enter to continue and connect the systems...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::cerr << "Could not connect to the YARP server\n";
    }

    yarp::os::RpcClient rpc;

    if (!rpc.open("/trajectory_generator_node/rpc:c") || !yarp::os::Network::connect(rpc.getName(), "/sceneReconstruction/rpc:s"))
    {
        std::cerr << "Could not establish connection with the reconstruction server\n";
    }

    roboticslab::SceneReconstructionIDL sceneReconstruction;
    sceneReconstruction.yarp().attachAsClient(rpc);

    if (jointTrajectory.empty())
    {
        std::cerr << "Error: no trajectory calculated in memory\n";
        return false;
    }

    std::string serialCmd = "G" + std::to_string(angle) + "T" + std::to_string(turningTime) + "\n";
    boost::asio::io_context ioSerial;
    boost::asio::serial_port serial(ioSerial);
    bool arduinoRet = false;

    try
    {
        serial.open("/dev/ttyACM0");
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        std::this_thread::sleep_for(std::chrono::seconds(2)); // wait for Arduino initialization
        arduinoRet = true;
        std::cout << "Serial port connected to the platform.\n";
    }
    catch (boost::system::system_error & e)
    {
        std::cerr << "Warning/Serial Port Error: " << e.what() << ". The script will continue without the physical platform.\n";
    }

    auto goalMsg = rl_cartesian_control_msgs::action::JointTrajectory::Goal();

    for (const auto & v : initialJointPositions)
    {
        goalMsg.position.push_back(v * KDL::deg2rad);
    }

    std::cout << "Moving to the safe point...\n";

    if (!sendJointTrajectoryGoal(goalMsg))
    {
        std::cerr << "Error: failed to move to the safe point\n";
        return false;
    }

    std::cout << "Robot reached the safe point\n";

    std::vector<float> firstJointPosition = jointTrajectory[0].data;

    std::cout << "Moving to the first point...\n";

    goalMsg.position.clear();

    for (const auto & v : firstJointPosition)
    {
        goalMsg.position.push_back(v);
    }

    if (!sendJointTrajectoryGoal(goalMsg))
    {
        std::cerr << "Error: failed to move to the first point\n";
        return false;
    }

    std::cout << "Robot reached the first point\n";

    sceneReconstruction.resume();

    if (std::abs(angle) == 360 || (std::abs(angle) == 180 && mode180 == 1))
    {
        doSimultaneous(serial, angle, turningTime);
    }
    else if (std::abs(angle) == 180 && mode180 == 2)
    {
        doSequential(serial, angle, turningTime);
    }

    sceneReconstruction.pause();

    if (arduinoRet)
    {
        char buffer[128];

        try
        {
            size_t n = serial.read_some(boost::asio::buffer(buffer, sizeof(buffer) - 1));
            buffer[n] = '\0';
            std::cout << "Arduino response: " << buffer << '\n';
        }
        catch (...) {}

        serial.close();
    }

    return true;
}

bool Robot::handleSolveIK(const std::vector<Point> & points, const TrajectoryGenerator & generator, bool debug)
{
    std::cout << "Press Enter to continue and calculate the inverse kinematics...\n";
    std::cin.get();

    KDL::Chain chain;

    for (const auto & dh : dhParams)
    {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh.a, dh.alpha * KDL::deg2rad, dh.d, dh.theta * KDL::deg2rad)));
    }

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(16.5, 0, 26))));
    //chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotY(M_PI_2), KDL::Vector(0.0, 0.0, 30.0))));

    KDL::ChainFkSolverPos_recursive fkSolverPos(chain);

    std::vector<double> initial_joints = generator.getInitialJointPositions();

    if (initial_joints.size() != 6)
    {
        std::cerr << "Error: the initial joint positions must contain 6 values\n";
        return false;
    }

    KDL::JntArray q(6);

    for (size_t i = 0; i < 6; ++i)
    {
        q(i) = initial_joints[i] * KDL::deg2rad;
    }

    if (debug)
    {
        KDL::Frame H_0_6;
        fkSolverPos.JntToCart(q, H_0_6);

        std::cout << "Number of joints: " << chain.getNrOfJoints() << '\n';
        std::cout << "Initial Cartesian position of the robot:\n";
        std::cout << "End-effector position (x): " << H_0_6.p.x() << '\n';
        std::cout << "End-effector position (y): " << H_0_6.p.y() << '\n';
        std::cout << "End-effector position (z): " << H_0_6.p.z() << '\n';

        std::cout << "Initial joint positions of the robot:\n";

        for (unsigned int i = 0; i < q.rows(); ++i)
        {
            std::cout << "Joint " << i + 1 << ": " << q(i) * KDL::rad2deg << " degrees\n";
        }
    }

    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);

    for (size_t i = 0; i < 6; ++i)
    {
        q_min(i) = jointLimits[i].min * KDL::deg2rad;
        q_max(i) = jointLimits[i].max * KDL::deg2rad;

        if (debug)
        {
            std::cout << "Joint " << i + 1 << ": ";
            std::cout << "Min: " << jointLimits[i].min << " degrees, ";
            std::cout << "Max: " << jointLimits[i].max << " degrees\n";
        }
    }

    KDL::ChainIkSolverVel_pinv ikSolverVel(chain);

    KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 200000, 0.001);

    jointTrajectory.clear();
    jointFlags.clear();

    if (points.empty())
    {
        std::cerr << "No trajectory points in memory.\n";
        return false;
    }

    double pointDuration = generator.calculatePointDuration();
    std::vector<double> minValues(6, std::numeric_limits<double>::max());
    std::vector<double> maxValues(6, std::numeric_limits<double>::lowest());
    bool allPointsReachable = true;
    KDL::JntArray temp = q;

    std::vector<double> centroid = generator.getCentroid();
    double startRad = generator.getStartPosition() * KDL::deg2rad;
    double inclinationRad = generator.getInclination() * KDL::deg2rad;

    std::cout << "Verifying trajectory in memory...\n\n";

    for (const auto & point : points)
    {
        double angle = startRad - std::atan2(point.y - centroid[1], point.x - centroid[0]);

        KDL::Frame desiredPose;
        desiredPose.p = KDL::Vector(point.x, point.y, point.z);
        desiredPose.M = KDL::Rotation::RotY(180 * KDL::deg2rad) * KDL::Rotation::RotZ(angle) * KDL::Rotation::RotY(inclinationRad);

        KDL::JntArray result(6);

        int ret = ikSolverPos.CartToJnt(temp, desiredPose, result);

        joint_flag jointFlag = REACHABLE;

        for (size_t i = 0; i < 6; ++i)
        {
            double joint_angle_deg = result(i) * KDL::rad2deg;
            minValues[i] = std::min(minValues[i], joint_angle_deg);
            maxValues[i] = std::max(maxValues[i], joint_angle_deg);
        }

        if (ret != 0)
        {
            bool exceedsLimits = false;

            for (size_t i = 0; i < 6; ++i)
            {
                if (result(i) <= q_min(i) || result(i) >= q_max(i))
                {
                    exceedsLimits = true;

                    if (debug)
                    {
                        std::cerr << "Error: Joint " << i + 1 << " exceeds limits. Value: "
                                  << result(i) * KDL::rad2deg << " degrees, Min limit: "
                                  << q_min(i) * KDL::rad2deg << " degrees, Max limit: "
                                  << q_max(i) * KDL::rad2deg << " degrees.\n";
                    }
                }
            }

            if (exceedsLimits && ret == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED)
            {
                std::cerr << "Unreachable point in trajectory: ("
                          << point.x << ", " << point.y << ", " << point.z << ")\n";

                allPointsReachable = false;
                jointFlag = UNREACHABLE;
            }
        }

        std::vector<float> current_joints_deg;

        for (unsigned int i = 0; i < result.rows(); ++i)
        {
            current_joints_deg.push_back(result(i));
        }

        std_msgs::msg::Float32MultiArray jointMsg;
        jointMsg.data = current_joints_deg;
        jointTrajectory.push_back(jointMsg);
        jointFlags.push_back(jointFlag);

        if (debug)
        {
            std::cout << "Point (" << point.x << ", " << point.y << ", " << point.z << ") OK.\n";
        }

        temp = result;
    }

    std::cout << "IK results:\n";
    std::cout << "===================================\n";

    for (size_t i = 0; i < 6; ++i)
    {
        std::cout << "Joint " << i + 1 << ":\n";
        std::cout << "  Min reached: " << minValues[i] << "°\n";
        std::cout << "  Max reached: " << maxValues[i] << "°\n";
        std::cout << "  Min limit: " << jointLimits[i].min << "°\n";
        std::cout << "  Max limit: " << jointLimits[i].max << "°\n";

        if (minValues[i] < jointLimits[i].min)
        {
            std::cout << "  Warning: Min reached value exceeds lower limit\n";
        }

        if (maxValues[i] > jointLimits[i].max)
        {
            std::cout << "  Warning: Max reached value exceeds upper limit\n";
        }
    }

    if (allPointsReachable)
    {
        std::cout << "All generated points are reachable.\n\n";
        return true;
    }
    else
    {
        std::cout << "The trajectory contains unreachable points. Adjust input parameters.\n";
        return false;
    }
}

bool Robot::handlePlotJoints(const TrajectoryGenerator & generator, bool debug, const std::string & filename)
{
    std::cout << "Press Enter to plot joint trajectories...\n";
    std::cin.get();
    std::cout << "Generating plot...\n";

    std::vector<std::vector<double>> data(6);

    if (jointTrajectory.empty())
    {
        std::cerr << "No joint trajectories found, run IK first\n";
        return false;
    }

    for (size_t i = 0; i < jointTrajectory.size(); ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            data[j].push_back(jointTrajectory[i].data[j]);
        }
    }

    double pointDuration = generator.calculatePointDuration();

    // assume all data vectors have the same length
    int n = data[0].size();

    if (debug)
    {
        std::cout << "Number of points: " << n << '\n';
    }

    std::vector<double> times(n);

    for (int i = 0; i < n; ++i)
    {
        times[i] = i * pointDuration;
    }

    if (debug)
    {
        std::cout << "Times vector:\n";

        for (const auto & t : times)
        {
            std::cout << t << " ";
        }

        std::cout << '\n';

        for (int i = 0; i < 6; ++i)
        {
            std::cout << "Data for joint " << i + 1 << ":\n";

            for (const auto & d : data[i])
            {
                std::cout << d << " ";
            }

            std::cout << '\n';
        }
    }

    // use NaN to represent unreachable points
    std::vector<std::vector<double>> dataWithNaN = data;

    for (int i = 0; i < n; ++i)
    {
        if (jointFlags[i] != Robot::REACHABLE)
        {
            for (int j = 0; j < 6; ++j)
            {
                dataWithNaN[j][i] = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    plt::figure();

    // ignore NaNs
    plt::named_plot("q1", times, dataWithNaN[0], "r-");
    plt::named_plot("q2", times, dataWithNaN[1], "g-");
    plt::named_plot("q3", times, dataWithNaN[2], "b-");
    plt::named_plot("q4", times, dataWithNaN[3], "c-");
    plt::named_plot("q5", times, dataWithNaN[4], "m-");
    plt::named_plot("q6", times, dataWithNaN[5], "y-");

    // plot unreachable points with scatter
    std::vector<double> unreachableTimes;
    std::vector<std::vector<double>> unreachableData(6);

    for (int i = 0; i < n; ++i)
    {
        if (jointFlags[i] != Robot::REACHABLE)
        {
            unreachableTimes.push_back(times[i]);

            for (int j = 0; j < 6; ++j)
            {
                unreachableData[j].push_back(data[j][i]);
            }
        }
    }

    plt::scatter(unreachableTimes, unreachableData[0], 10.0, { {"color", "r"} });
    plt::scatter(unreachableTimes, unreachableData[1], 10.0, { {"color", "g"} });
    plt::scatter(unreachableTimes, unreachableData[2], 10.0, { {"color", "b"} });
    plt::scatter(unreachableTimes, unreachableData[3], 10.0, { {"color", "c"} });
    plt::scatter(unreachableTimes, unreachableData[4], 10.0, { {"color", "m"} });
    plt::scatter(unreachableTimes, unreachableData[5], 10.0, { {"color", "y"} });

    std::vector<std::string> limit_colors = { "r", "g", "b", "c", "m", "y" };

    for (int i = 0; i < 6; ++i)
    {
        std::string color = limit_colors[i];

        plt::axhline(jointLimits[i].min, 0, times.back(), { {"color", color}, {"linestyle", ":"}, {"linewidth", "0.5"} });
        plt::axhline(jointLimits[i].max, 0, times.back(), { {"color", color}, {"linestyle", "-."}, {"linewidth", "0.5"} });

        // avoid decimals
        std::string minLabel = std::to_string(static_cast<int>(jointLimits[i].min)) + "°";
        std::string maxLabel = std::to_string(static_cast<int>(jointLimits[i].max)) + "°";

        plt::text(times.back() * 1, jointLimits[i].min, minLabel);
        plt::text(times.back() * 1, jointLimits[i].max, maxLabel);

        double minValue = *std::min_element(data[i].begin(), data[i].end());
        double maxValue = *std::max_element(data[i].begin(), data[i].end());

        int minIndex = std::distance(data[i].begin(), std::min_element(data[i].begin(), data[i].end()));
        int maxIndex = std::distance(data[i].begin(), std::max_element(data[i].begin(), data[i].end()));

        plt::scatter(std::vector<double>{times[minIndex]}, std::vector<double>{minValue}, 20, {{"color", color}, {"marker", "x"}});
        plt::scatter(std::vector<double>{times[maxIndex]}, std::vector<double>{maxValue}, 20, {{"color", color}, {"marker", "s"}});
    }

    plt::named_plot("Maximum limit", std::vector<double>{}, std::vector<double>{}, "k-.");
    plt::named_plot("Minimum limit", std::vector<double>{}, std::vector<double>{}, "k:");
    plt::named_plot("Unreachable points", std::vector<double>{}, std::vector<double>{}, "k.");
    plt::named_plot("Maximum values", std::vector<double>{}, std::vector<double>{}, "ks");
    plt::named_plot("Minimum values", std::vector<double>{}, std::vector<double>{}, "kx");

    plt::xlabel("Time (s)");
    plt::ylabel("Joint value (deg)");
    plt::title("Temporal evolution of joint trajectories");

    plt::legend();

    if (!filename.empty())
    {
        plt::save(filename);
        std::cout << "Plot saved in: " << filename << '\n';
    }
    else
    {
        std::cout << "Showing plot " << filename << '\n';
        plt::show();
    }

    plt::close();

    return true;
}
