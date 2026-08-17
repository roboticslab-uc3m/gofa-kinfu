#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>
#include <vector>

#include <abb_libegm/egm_trajectory_interface.h>

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

    Robot();
    Robot(const std::string & filename);

    bool handleExecute(double speed, double radius, std::vector<double> initialJointPositions);
    bool doSimultaneous(abb::egm::EGMTrajectoryInterface & egm_interface, boost::asio::serial_port & serial, int angle, int turningTime);
    bool doSequential(abb::egm::EGMTrajectoryInterface & egm_interface, boost::asio::serial_port & serial, int angle, int turningTime);

    bool handleSolveIK(const std::vector<Point> & points, const TrajectoryGenerator & trajectory, bool debug);
    bool handlePlotJoints(const TrajectoryGenerator & trajectory, bool debug, const std::string & filename = "");

private:
    void parseDHParameters(const std::string & filename);
    void parseJointLimits(const std::string & filename);

    std::vector<DHParameters> dh_params;
    std::vector<JointLimits> jointLimits;
    std::vector<std::vector<double>> jointTrajectory;
    std::vector<joint_flag> jointFlags;

    abb::egm::wrapper::trajectory::TrajectoryGoal goal;
};

#endif // __ROBOT_H__
