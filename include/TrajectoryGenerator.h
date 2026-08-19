#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>
#include <string>
#include <vector>

struct Point
{
    double x;
    double y;
    double z;
};

class TrajectoryGenerator
{
public:
    double calculatePointDuration() const;
    bool handleGenerate(bool debug);
    void handlePlot(const std::string & filename = "");

    const std::vector<double> & getCentroid() const { return centroid; }
    double getRadius() const { return radius; }
    double getHeight() const { return height; }
    double getInclination() const { return inclination; }
    int getResolution() const { return resolution; }
    double getSpeed() const { return speed; }
    double getStartPosition() const { return startPosition; }
    double getEndPosition() const { return endPosition; }
    const std::vector<double> & getInitialJointPositions() const { return initialJointPositions; }
    const std::vector<Point> & getGeneratedPoints() const { return generatedPoints; }

    static std::unique_ptr<TrajectoryGenerator> parseInputParameters(const std::string & filename);
    static std::unique_ptr<TrajectoryGenerator> readFromConsole();

private:
    TrajectoryGenerator() = default;
    TrajectoryGenerator(const TrajectoryGenerator &) = delete;
    TrajectoryGenerator & operator=(const TrajectoryGenerator &) = delete;

    std::vector<Point> generateCircularTrajectory();

    std::vector<double> centroid {0.0, 0.0, 0.0};
    std::vector<double> initialJointPositions;
    std::vector<Point> generatedPoints;

    double radius {0.0};
    double height {0.0};
    double inclination {0.0}; // [rad]
    int resolution {0};
    double speed {0.0};
    double startPosition {0.0}; // [rad]
    double endPosition {0.0}; // [rad]
};

#endif // __TRAJECTORY_GENERATOR_H__
