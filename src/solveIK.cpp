#include "functions.h"
#include "trajectory_generator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
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
#include <abb_libegm/egm_trajectory_interface.h> // Incluye la interfaz de trayectorias EGM para la comunicación con el robot ABB
#include <yaml-cpp/yaml.h>

bool handleSolveIK(const std::string& robotParams, const std::string& pointsFile, const std::string& outputFile, const InputParameters& input_params, const std::vector<JointLimits>& joint_limits, bool debug) {
    std::cout << "Presione Enter para continuar y calcular la cinemática inversa..." << std::endl;
    std::cin.get();
    // Definición de la cadena cinemática usando los parámetros DH del archivo YAML
    KDL::Chain chain;
    // Leer los parámetros de dh desde el archivo YAML
    std::vector<DHParameters> dh_params;
    parseDHParameters(robotParams, dh_params);
    for (const auto& dh : dh_params) {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh.a, dh.alpha * KDL::deg2rad, dh.d, dh.theta * KDL::deg2rad)));
    }

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0,0,30))));

    // Solver de cinemática directa
    KDL::ChainFkSolverPos_recursive FKSolverPos(chain);

    // Posiciones articulares iniciales
    std::vector<double> initial_positions = input_params.initial_joint_positions;

    if (initial_positions.size() != 6) {
        std::cerr << "Error: Las posiciones iniciales de las articulaciones deben contener 6 valores.\n";
        return false;
    }

    //Posición inicial en coordenadas articulares
    KDL::JntArray q(6);
    for (size_t i = 0; i < 6; ++i) {
        q(i) = initial_positions[i] * KDL::deg2rad;
    }

    KDL::Frame H_0_6;
    FKSolverPos.JntToCart(q, H_0_6); //Calcula pose en cartesianas del extremo a partir de las posiciones de las articulaciones

    if (debug) {
        std::cout << "Número de articulaciones: " << chain.getNrOfJoints() << std::endl; //Muesta por pantalla el número de articulaciones en la cadena cinemática
        //Muestra por pantalla las coordenadas x, y, z de la posición del extremo del robot
        std::cout << "Posición cartesiana inicial del robot:" << std::endl;
        std::cout << "Posición del extremo (x): " << H_0_6.p.x() << std::endl;
        std::cout << "Posición del extremo (y): " << H_0_6.p.y() << std::endl;
        std::cout << "Posición del extremo (z): " << H_0_6.p.z() << std::endl;

        // Muestra por pantalla las coordenadas articulares del extremo del robot
        std::cout << "Posición articular inicial del robot:" << std::endl;
        for (unsigned int i = 0; i < q.rows(); ++i) {
            std::cout << "Articulación " << i + 1 << ": " << q(i) * KDL::rad2deg << " grados" << std::endl;
        }
    }

    //Límites de las articulaciones extraídos del archivo YAML
    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);

    // Asignar los limites a las 6 articulaciones
    for (size_t i = 0; i < 6; ++i) {
        q_min(i) = joint_limits[i].min * KDL::deg2rad;
        q_max(i) = joint_limits[i].max * KDL::deg2rad;

        if (debug) {
            // Imprimir los límites para cada articulación
            std::cout << "Articulación " << i + 1 << ": ";
            std::cout << "Min: " << joint_limits[i].min << " grados, ";
            std::cout << "Max: " << joint_limits[i].max << " grados" << std::endl;
        }
    }

    // Inicializar los valores mínimos y máximos para cada articulación
    std::vector<double> min_values(6, std::numeric_limits<double>::max());
    std::vector<double> max_values(6, std::numeric_limits<double>::lowest());

    //Solver de cinemática diferencial inversa
    KDL::ChainIkSolverVel_pinv IKSolverVel(chain);

    // Solver de cinemática inversa
    KDL::ChainIkSolverPos_NR_JL IKSolverPos(chain, q_min, q_max, FKSolverPos, IKSolverVel, 200000, 0.001);

    // Abrir archivo para leer los puntos de la trayectoria
    std::ifstream infile(pointsFile);
    if (!infile.is_open()) {
        std::cerr << "No se pudo abrir el archivo con los puntos de la trayectoria.\n";
        return false;
    }

    // Leer los puntos de la trayectoria desde el archivo
    std::vector<Point> trajectory_points;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream stream(line);
        Point p;
        if (stream >> p.x >> p.y >> p.z) {
            trajectory_points.push_back(p);
        }
        else {
            std::cerr << "Formato de línea incorrecto: " << line << std::endl;
        }
    }
    infile.close();

    if (trajectory_points.empty()) {
        std::cerr << "No se generaron puntos de trayectoria." << std::endl;
        return false;
    }

    // Abrir archivo para guardar las posiciones articulares calculadas por KDL
    std::ofstream kdl_file(outputFile);
    if (!kdl_file.is_open()) {
        std::cerr << "No se pudo abrir el archivo para escribir las posiciones articulares calculadas por KDL.\n";
        return false;
    }

    // Crear la trayectoria para enviarla al robot
    abb::egm::wrapper::trajectory::TrajectoryGoal trajectory; // Crea la trayectoria que se enviará al robot
    bool all_points_reachable = true;

    KDL::JntArray temp = q; // Variable temporal

    // Generar la trayectoria y calcular la duración de cada punto
    TrajectoryGenerator generator(
        input_params.centroid,
        input_params.radius,
        input_params.height,
        input_params.inclination,
        input_params.resolution,
        input_params.start_position,
        input_params.end_position,
        input_params.speed
    );

    double point_duration = generator.calculatePointDuration();

    std::cout << "Verificando trayectoria...\n" << std::endl;
    for (const auto& point : trajectory_points) { // Itera sobre cada punto del vector de la trayectoria

        abb::egm::wrapper::trajectory::PointGoal* traj_point = trajectory.add_points(); // Añade un punto a la trayectoria

        double angle = input_params.start_position* KDL::deg2rad - atan2(point.y - input_params.centroid[1], point.x - input_params.centroid[0]);  // Ángulo inicial en radianes

        // Verificar cada punto de la trayectoria
        KDL::Frame desiredPose;
        desiredPose.p = KDL::Vector(point.x, point.y, point.z);
        // Ajustar la rotación usando la secuencia de Euler YZY
        desiredPose.M = KDL::Rotation::RotY(180 * KDL::deg2rad) * KDL::Rotation::RotZ(angle) * KDL::Rotation::RotY(input_params.inclination * KDL::deg2rad);

        KDL::JntArray result(6);

        int ret = IKSolverPos.CartToJnt(temp, desiredPose, result);

        // Variable para indicar si hubo error (1:error, 0:no error)
        int error_indicator = 0;

        // Actualizar valores mínimos y máximos para cada articulación incluso si el punto es inalcanzable
        for (size_t i = 0; i < 6; ++i) {
            double joint_angle_deg = result(i) * KDL::rad2deg;
            min_values[i] = std::min(min_values[i], joint_angle_deg);
            max_values[i] = std::max(max_values[i], joint_angle_deg);
        }

        if (ret != 0) { // Si algún punto de la trayectoria no es alcanzable
            /*all_points_reachable = false;
            error_indicator = 1;

            std::cout << "Punto inalcanzable en la trayectoria: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;*/

            /*if (ret == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {
                std::cout << "Punto inalcanzable en la trayectoria: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            }
            else {
                std::cout << "Punto inalcanzable en la trayectoria (otro error): (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            }*/

            // Comprobar si alguna articulación excede los límites
            bool exceeds_limits = false;
            for (size_t i = 0; i < 6; ++i) {
                if (result(i) <= q_min(i) || result(i) >= q_max(i)) {
                    exceeds_limits = true;
                    if (debug) {
                        std::cerr << "Error: La articulación " << i + 1 << " excede los límites. Valor: "
                            << result(i) * KDL::rad2deg << " grados, Límite mínimo: "
                            << q_min(i) * KDL::rad2deg << " grados, Límite máximo: "
                            << q_max(i) * KDL::rad2deg << " grados.\n";
                    }
                }
            }

            if (exceeds_limits) {
                if (ret == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {
                    std::cerr << "Punto inalcanzable en la trayectoria: ("
                        << point.x << ", " << point.y << ", " << point.z << ")\n";
                    all_points_reachable = false;
                    error_indicator = 1;
                }
            }
        }

        // Configurar las posiciones articulares en grados
        for (unsigned int i = 0; i < result.rows(); ++i) {
            traj_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(result(i) * KDL::rad2deg);
        }

        // Agregar duración
        traj_point->set_duration(point_duration);

        if (debug) {
            // Imprimir las posiciones actuales de las articulaciones
            std::cout << "Posiciones de las articulaciones para el punto (" << point.x << ", " << point.y << ", " << point.z << ") : " << std::endl;
            for (unsigned int i = 0; i < result.rows(); ++i) {
                std::cout << "Articulación " << i + 1 << ": " << result(i) * KDL::rad2deg << " grados" << std::endl; // Convertir de radianes a grados
            }
        }

        // Guardar las posiciones articulares en el archivo y el indicador del error
        for (unsigned int i = 0; i < result.rows(); ++i) {
            kdl_file << result(i) * KDL::rad2deg << " ";
        }
        kdl_file << error_indicator << "\n";

        temp = result;
    }

    // Cerrar el archivo
    kdl_file.close();

    // Mostrar resultados mínimos y máximos
    std::cout << "Resultados de la IK:\n";
    std::cout << "===================================\n";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << "Articulación " << i + 1 << ":\n";
        std::cout << "  Mínimo alcanzado: " << min_values[i] << "°\n";
        std::cout << "  Máximo alcanzado: " << max_values[i] << "°\n";
        std::cout << "  Límite mínimo: " << joint_limits[i].min << "°\n";
        std::cout << "  Límite máximo: " << joint_limits[i].max << "°\n";

        // Comparación con los límites
        if (min_values[i] < joint_limits[i].min) {
            std::cout << "  Advertencia: El valor mínimo alcanzado excede el límite inferior\n";
        }
        if (max_values[i] > joint_limits[i].max) {
            std::cout << "  Advertencia: El valor máximo alcanzado excede el límite superior\n";
        }
    }

    if (all_points_reachable) {
        std::cout << "Todos los puntos generados son alcanzables.\n" << std::endl;
    }
    else {// Si hay algún punto inalcanzable, mensaje de error y terminar el programa
        std::cout << "La trayectoria contiene puntos inalcanzables. Ajusta los parámetros de entrada." << std::endl;
        return 1;
    }

    return true;
}
