#include <cmath>

#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <boost/program_options.hpp>
#include <matplotlibcpp.h>
#include <yaml-cpp/yaml.h>

#include "TrajectoryGenerator.h"
#include "Robot.h"

namespace po = boost::program_options;

std::ostream & operator<<(std::ostream & os, const Point & point)
{
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}

void loadRobotData(const std::string & filename)
{
    try
    {
        YAML::Node robot = YAML::LoadFile(filename);

        std::cout << "Robot configuration read from file: " << filename << std::endl;
        std::cout << "\n===================================\n";

        // Mostrar el nombre del robot
        std::cout << "Robot:\n";
        std::cout << "  " << robot["robot"].as<std::string>() << std::endl;

        std::cout << "===================================\n";

        // Mostrar los parámetros DH en formato tabla
        std::cout << "DH parameters:\n";
        std::cout << std::setw(8) << "theta" << std::setw(8) << "D" << std::setw(12) << "A" << std::setw(8) << "alpha" << "\n";
        std::cout << "  ----------------------------------------\n";

        for (const auto & param : robot["dh_parameters"])
        {
            std::cout << std::setw(8) << param["theta"].as<double>()
                      << std::setw(8) << param["d"].as<int>()
                      << std::setw(8) << param["a"].as<double>()
                      << std::setw(8) << param["alpha"].as<int>() << std::endl;
        }

        std::cout << "===================================\n";

        // Mostrar los límites de las articulaciones en formato tabla
        std::cout << "Joint limits:\n";
        std::cout << std::setw(8) << "min" << std::setw(8) << "max" << "\n";
        std::cout << "  ------------\n";

        for (const auto & limit : robot["joint_limits"])
        {
            std::cout << std::setw(8) << limit["min"].as<double>()
                      << std::setw(8) << limit["max"].as<double>() << std::endl;
        }

        std::cout << "===================================\n";
    }
    catch (const YAML::Exception & e)
    {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

int main(int argc, char ** argv)
{
    Robot robot;
    std::unique_ptr<TrajectoryGenerator> generator;

    try {
        po::options_description desc("Supported options");

        desc.add_options()
            ("help", "Show help message\n")
            ("debug", "Enable debug mode")
            ("generate", po::value<std::string>(), "Generate parameters\n")
            ("plot", po::value<std::string>()->implicit_value(""), "Plot points\n")
            ("robot", po::value<std::string>(), "Robot YAML file\n")
            ("solveIK", "Execute inverse kinematics\n")
            ("plot_joints", po::value<std::string>()->implicit_value(""), "Plot joints\n")
            ("execute", "Execute movements\n");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        bool debug = vm.count("debug");

        if (vm.count("help") || vm.empty())
        {
            std::cout << desc << std::endl;
            return 1;
        }

        if (vm.count("generate"))
        {
            std::string gen = vm["generate"].as<std::string>();

            if (gen == "input")
            {
                generator = TrajectoryGenerator::readFromConsole();
            }
            else
            {
                generator = TrajectoryGenerator::parseInputParameters(gen);
            }

            if (!generator->handleGenerate(debug))
            {
                throw std::invalid_argument("trajectory generation failed");
            }
        }
        else if (vm.count("plot_joints") || vm.count("solveIK"))
        {
            // Carga por defecto para que input_params no esté vacío
            generator->parseInputParameters("input_parameters.yml");
        }

        if (vm.count("plot"))
        {
            generator->handlePlot(vm["plot"].as<std::string>());
        }

        // Sacamos la llamada de los condicionales para que se ejecute siempre que esté el flag
        if (vm.count("robot"))
        {
            std::string robot_file = vm["robot"].as<std::string>();
            robot = Robot(robot_file); // Se carga siempre aquí

            // Si solo se quiere visualizar la info del robot (sin otras acciones)
            if (vm.size() == 1 || (vm.size() == 2 && debug))
            {
                loadRobotData(robot_file);
            }
        }

        if (vm.count("solveIK"))
        {
            if (!vm.count("robot"))
            {
                std::cerr << "Error: --solveIK requires --robot\n";
                return 1;
            }

            robot.handleSolveIK(generator->getGeneratedPoints(), *generator, debug);
        }

        if (vm.count("plot_joints"))
        {
            robot.handlePlotJoints(*generator, debug, vm["plot_joints"].as<std::string>());
        }

        if (vm.count("execute"))
        {
            robot.handleExecute(generator->getSpeed(), generator->getRadius(), generator->getInitialJointPositions());
        }
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
