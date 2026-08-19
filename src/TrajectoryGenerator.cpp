#include "TrajectoryGenerator.h"

#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

constexpr auto RAD2DEG = 180.0 / M_PI;
constexpr auto DEG2RAD = M_PI / 180.0;

std::unique_ptr<TrajectoryGenerator> TrajectoryGenerator::parseInputParameters(const std::string & filename)
{
    // can't use std::make_unique here due to the private ctor
    std::unique_ptr<TrajectoryGenerator> generator(new TrajectoryGenerator());

    YAML::Node config = YAML::LoadFile(filename);
    const auto & params = config["input_parameters"];

    generator->centroid = params["centroid"].as<std::vector<double>>();
    generator->radius = params["radius"].as<double>();
    generator->height = params["height"].as<double>();
    generator->inclination = params["inclination"].as<double>() * DEG2RAD;
    generator->resolution = params["resolution"].as<int>();
    generator->speed = params["speed"].as<double>();
    generator->startPosition = params["start_position"].as<double>() * DEG2RAD;
    generator->endPosition = params["end_position"].as<double>() * DEG2RAD;
    generator->initialJointPositions = params["initial_joint_positions"].as<std::vector<double>>();

    return generator;
}

std::unique_ptr<TrajectoryGenerator> TrajectoryGenerator::readFromConsole()
{
    // can't use std::make_unique here due to the private ctor
    std::unique_ptr<TrajectoryGenerator> generator(new TrajectoryGenerator());

    generator->centroid.resize(3);

    std::cout << "Enter the circumference center (cx, cy, cz): ";
    std::cin >> generator->centroid[0] >> generator->centroid[1] >> generator->centroid[2];
    std::cout << "Enter the circumference radius: ";
    std::cin >> generator->radius;
    std::cout << "Enter the height: ";
    std::cin >> generator->height;
    std::cout << "Enter the inclination (degrees): ";
    std::cin >> generator->inclination;
    std::cout << "Enter the resolution (number of points): ";
    std::cin >> generator->resolution;
    std::cout << "Enter the start position (degrees): ";
    std::cin >> generator->startPosition;
    std::cout << "Enter the end position (degrees): ";
    std::cin >> generator->endPosition;
    std::cout << "Enter the speed (mm/s): ";
    std::cin >> generator->speed;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return generator;
}

std::vector<Point> TrajectoryGenerator::generateCircularTrajectory()
{
    std::vector<Point> trajectory;
    double angle_increment = (endPosition - startPosition) / resolution;

    for (int i = 0; i < resolution; ++i)
    {
        double t = startPosition + i * angle_increment;
        double x = radius * std::cos(t);
        double y = radius * std::sin(t);
        double z = height;

        Point p {centroid[0] + x, centroid[1] + y, centroid[2] + z};
        trajectory.push_back(p);
    }

    return trajectory;
}

double TrajectoryGenerator::calculatePointDuration() const
{
    double circumference = (endPosition - startPosition) * radius;
    double totalDuration = circumference / speed;
    return totalDuration / resolution;
}

bool TrajectoryGenerator::handleGenerate(bool debug)
{
    std::cout << "Generating a trajectory from the following stored parameters:\n";
    std::cout << "Circumference center: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")\n";
    std::cout << "Circumference radius: " << radius << '\n';
    std::cout << "Height: " << height << '\n';
    std::cout << "Inclination (degrees): " << inclination * RAD2DEG << '\n';
    std::cout << "Resolution (number of points): " << resolution << '\n';
    std::cout << "Start position (degrees): " << startPosition * RAD2DEG << '\n';
    std::cout << "End position (degrees): " << endPosition * RAD2DEG << '\n';
    std::cout << "Speed (mm/s): " << speed << '\n';

    generatedPoints = generateCircularTrajectory();

    if (generatedPoints.empty())
    {
        std::cerr << "Error: empty trajectory\n";
        return false;
    }

    double pointDuration = calculatePointDuration();

    if (debug)
    {
        std::cout << "Point duration: " << pointDuration << " seconds\n";
    }

    if (debug)
    {
        std::cout << "Press Enter to print generated points...\n";
        std::cin.get();
        std::cout << "Generated points:\n";

        for (size_t i = 0; i < generatedPoints.size(); ++i)
        {
            std::cout << "Point " << i << ": ("
                      << generatedPoints[i].x << ", "
                      << generatedPoints[i].y << ", "
                      << generatedPoints[i].z << ")\n";
        }
    }

    std::ofstream outFile("trajectory_points.txt");

    if (!outFile.is_open())
    {
        std::cerr << "Error: unable to open file to write points to\n";
        return false;
    }

    for (const auto & point : generatedPoints)
    {
        outFile << point.x << " " << point.y << " " << point.z << '\n';
    }

    outFile.close();

    return true;
}

void TrajectoryGenerator::handlePlot(const std::string & filename)
{
    std::cout << "Press Enter to generate plot...\n";
    std::cin.get();
    std::cout << "Generating plot...\n";

    std::vector<double> x, y, z;

    for (const auto & point : generatedPoints)
    {
        x.push_back(point.x);
        y.push_back(point.y);
        z.push_back(point.z);
    }

    plt::figure();
    plt::named_plot("Trajectory points", x, y, "b.");
    plt::scatter(x, y, 0.1);

    std::ostringstream title;
    title << "Generated Cartesian trajectory (height = " << z[0] << " mm)";

    plt::title(title.str());
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::axis("equal");
    plt::legend();

    if (!filename.empty())
    {
        plt::save(filename);
        std::cout << "Plot exported to: " << filename << '\n';
    }
    else
    {
        std::cout << "Loading plot " << filename << '\n';
        plt::show();
    }

    plt::close();
}
