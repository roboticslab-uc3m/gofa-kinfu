// Integra la generación de trayectorias con la librería abb_libegm
#define _USE_MATH_DEFINES
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include "trajectory_generator.h" // Incluye la definición de la clase TrajectoryGenerator que genera la trayectoria circular
#include "parser.h"
#include "functions.h"
#include <abb_libegm/egm_trajectory_interface.h> // Incluye la interfaz de trayectorias EGM para la comunicación con el robot ABB
#include <abb_libegm/egm_controller_interface.h> // Para integrar la lectura de posiciones reales del robot
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/utilities/utility.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <matplotlibcpp.h>
#include <yaml-cpp/yaml.h>
#include <iomanip>


namespace plt = matplotlibcpp;
namespace po = boost::program_options;

// Sobrecarga de operator<< para imprimir Point
std::ostream& operator<<(std::ostream& os, const Point& point) {
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}

// Función para cargar y mostrar los datos del robot desde un archivo YAML
void loadRobotData(const std::string& filename) {
    try {
        YAML::Node robot = YAML::LoadFile(filename);
        std::cout << "Datos del robot leídos del archivo: " << filename << std::endl;
        std::cout << "\n===================================\n";

        // Mostrar el nombre del robot
        std::cout << "Robot:\n";
        std::cout << "  " << robot["robot"].as<std::string>() << std::endl;

        std::cout << "===================================\n";

        // Mostrar los parámetros DH en formato tabla
        std::cout << "DH Parameters:\n";
        std::cout << std::setw(8) << "a" << std::setw(8) << "alpha" << std::setw(12) << "d" << std::setw(8) << "theta" << "\n";
        std::cout << "  ----------------------------------------\n";
        for (const auto& param : robot["dh_parameters"]) {
            std::cout << std::setw(8) << param["a"].as<double>()
                << std::setw(8) << param["alpha"].as<int>()
                << std::setw(12) << param["d"].as<double>()
                << std::setw(8) << param["theta"].as<int>() << std::endl;
        }

        std::cout << "===================================\n";

        // Mostrar los límites de las articulaciones en formato tabla
        std::cout << "Joint Limits:\n";
        std::cout << std::setw(8) << "min" << std::setw(8) << "max" << "\n";
        std::cout << "  ------------\n";
        for (const auto& limit : robot["joint_limits"]) {
            std::cout << std::setw(8) << limit["min"].as<double>()
                << std::setw(8) << limit["max"].as<double>() << std::endl;
        }

        std::cout << "===================================\n";
    }
    catch (const YAML::Exception& e) {
        std::cerr << "Error al leer el archivo YAML: " << e.what() << std::endl;
    }
}

InputParameters input_params; // Variable para almacenar los datos de entrada parseados del archivo YAML
std::vector<JointLimits> joint_limits; // Variable para almacenar los límites de las articulaciones

int main(int argc, char** argv) {

    try {
        // Declarar las opciones permitidas del programa
        po::options_description desc("Opciones permitidas");
        desc.add_options()
            ("help", "Mostrar mensaje de ayuda\n")
            ("debug", "Habilitar el modo debug")
            ("generate", po::value<std::string>(), "Generar parámetros ('input' para entrada manual o 'input_parameters.yml')\n")
            ("plot", po::value<std::string>()->implicit_value(""), "Mostrar el gráfico con los puntos generados por pantalla o guardarlo en un archivo si se proporciona un nombre de archivo(.png)\n")
            ("robot", po::value<std::string>(), "Seleccionar robot, proporcionar archivo YAML con los parámetros del robot\n")
            ("solveIK", po::value<std::string>(), "Ejecutar la cinemática inversa, proporcionar archivo de puntos generados(.txt)\n")
            ("plot_joints", po::value<std::string>()->implicit_value(""), "Mostrar el gráfico con los grados de las articulaciones por pantalla o guardarlo en un archivo si se proporciona un nombre de archivo(.png)\n")
            ("execute", po::value<std::string>(), "Eejcutar movimientos del robot, proporcionar la solución de la cinemática inversa(.txt)\n")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        bool debug = false;

        if (vm.count("help") || vm.empty()) {
            std::cout << desc << std::endl;
            return 1;
        }

        if (vm.count("debug")) {
            debug = true;
        }

        if (vm.count("generate")) {
            std::string generate_option = vm["generate"].as<std::string>();
            if (generate_option == "input") {
                readParameters(input_params); // Leer los parámetros de entrada desde el teclado
            }
            else {
                parseInputParameters(generate_option, input_params); // Leer los parámetros de entrada desde el archivo YAML
            }
            std::vector <Point> trajectory = handleGenerate(input_params, debug);
        }

        if (vm.count("plot")) {
            std::string plot_option = vm["plot"].as<std::string>();
            // Mostrar el gráfico en ambos casos
            if (plot_option.empty()) {
                // Solo mostrar el gráfico
                handlePlot("");
            }
            else {
                // Guardar el gráfico
                handlePlot(plot_option);
            }
        }

        // Si se proporciona la opción --robot, cargar y mostrar los datos del robot
        if (vm.count("robot") && !vm.count("generate") && !vm.count("solveIK") && !vm.count("plot_joints") && !vm.count("execute") && !vm.count("plot")) {
            std::string robot_file = vm["robot"].as<std::string>();
            loadRobotData(robot_file);
        }

        if (vm.count("robot") && vm.count("solveIK")) {
            std::string robotParams = vm["robot"].as<std::string>();
            std::string pointsFile = vm["solveIK"].as<std::string>();
            std::string outputFile = "kdl_joint_positions.txt"; // Nombre del archivo de salida

            parseJointLimits(robotParams, joint_limits);

            if (handleSolveIK(robotParams, pointsFile, outputFile, input_params, joint_limits, debug)) {
                std::cout << "La cinemática inversa se resolvió y se guardó en " << outputFile << std::endl;
            }
            else {
                std::cerr << "No se pudo resolver la cinemática inversa" << std::endl;
                return 1;
            }
        }
        else if (vm.count("solveIK")) {
            std::cerr << "Error: --solveIK requiere los parámetros --robot\n";
            return 1;
        }

        if (vm.count("plot_joints")) {
            std::string plot_joints_option = vm["plot_joints"].as<std::string>();

            if (plot_joints_option.empty()) {
                // Solo mostrar el gráfico
                handlePlotJoints(input_params, joint_limits, debug, "");
            }
            else {
                // Guardar el gráfico
                handlePlotJoints(input_params, joint_limits, debug, plot_joints_option);
            }
        }

        if (vm.count("execute")) {
            std::string execute_option = vm["execute"].as<std::string>();
            handleExecute(execute_option, input_params);
        }
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    catch (...) {
        std::cerr << "Excepción de tipo desconocido!\n";
    }


    return 0;
}
