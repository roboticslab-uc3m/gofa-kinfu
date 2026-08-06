#include "functions.h"
#include "trajectory_generator.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>


// Función para generar y visualizar la trayectoria
std::vector <Point> handleGenerate(const InputParameters& input_params, bool debug) {
    std::cout << "Generando trayectoria con los siguientes parámetros:" << std::endl;
    // Imprimir los parámetros de entrada leídos del archivo YAML
    std::cout << "Centro de la circunferencia: (" << input_params.centroid[0] << ", " << input_params.centroid[1] << ", " << input_params.centroid[2] << ")\n";
    std::cout << "Radio de la circunferencia: " << input_params.radius << "\n";
    std::cout << "Altura: " << input_params.height << "\n";
    std::cout << "Inclinación vertical en grados: " << input_params.inclination << "\n";
    std::cout << "Resolución (número de puntos): " << input_params.resolution << "\n";
    std::cout << "Posición inicial (grados): " << input_params.start_position << "\n";
    std::cout << "Posición final (grados): " << input_params.end_position << "\n";
    std::cout << "Velocidad deseada (mm/s): " << input_params.speed << "\n";

    // Generador de trayectoria
    TrajectoryGenerator generator(
        input_params.centroid,
        input_params.radius,
        input_params.height,
        input_params.inclination,
        input_params.resolution,
        input_params.start_position,
        input_params.end_position,
        input_params.speed
    ); // Inicializa la clase con los parámetros de entrada leídos del archivo YAML

    auto trajectory_points = generator.generateCircularTrajectory();// Obtiene puntos de la circunferencia llamando a generateCircularTrajectory

    // Verificar si se generaron los puntos de la trayectoria
    if (trajectory_points.empty()) {
        std::cerr << "Error: No se generaron puntos de la trayectoria.\n";
        return trajectory_points;
    }

    // Calcular y mostrar la duración por punto
    double point_duration = generator.calculatePointDuration();

    if (debug) {
        std::cout << "Duración por punto: " << point_duration << " segundos" << std::endl;
    }

    // Imprimir los puntos generados para verificación
    if (debug) {
        // Pausa para el usuario antes de imprimir los puntos generados
        std::cout << "Presione Enter para continuar y ver los puntos generados..." << std::endl;
        std::cin.get();
        std::cout << "Puntos generados:" << std::endl;
        for (size_t i = 0; i < trajectory_points.size(); ++i) {
        std::cout << "Punto " << i << ": (" << trajectory_points[i].x << ", " << trajectory_points[i].y << ", " << trajectory_points[i].z << ")" << std::endl;
        }
    }

    // Guardar los puntos generados en un archivo de texto
    std::ofstream outFile("trajectory_points.txt");
    if (!outFile.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo para escribir los puntos de la trayectoria.\n";
        return trajectory_points;
    }

    for (const auto& point : trajectory_points) {
        outFile << point.x << " " << point.y << " " << point.z << std::endl;
    }

    outFile.close();

    return trajectory_points;
}

// Función para leer los parámetros de entrada desde el teclado
void readParameters(InputParameters& input_params) {
    // Inicializar los vectores con 3 elementos
    input_params.centroid.resize(3);  // Asegurarse de que el vector tenga espacio para 3 elementos

    std::cout << "Ingrese el centro de la circunferencia (cx, cy, cz): ";
    std::cin >> input_params.centroid[0] >> input_params.centroid[1] >> input_params.centroid[2];
    std::cout << "Ingrese el radio de la circunferencia: ";
    std::cin >> input_params.radius;
    std::cout << "Ingrese la altura: ";
    std::cin >> input_params.height;
    std::cout << "Ingrese la inclinación vertical en grados: ";
    std::cin >> input_params.inclination;
    std::cout << "Ingrese la resolución (número de puntos): ";
    std::cin >> input_params.resolution;
    std::cout << "Ingrese la posición de inicio en grados (0-360): ";
    std::cin >> input_params.start_position;
    std::cout << "Ingrese la posición de fin en grados (0-360): ";
    std::cin >> input_params.end_position;
    std::cout << "Ingrese la velocidad deseada (mm/s): ";
    std::cin >> input_params.speed;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}
