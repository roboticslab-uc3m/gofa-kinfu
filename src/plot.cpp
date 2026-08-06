#include "functions.h"
#include "trajectory_generator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

// Función para cargar los puntos desde el archivo txt
bool loadDataFromFile(const std::string& filename, std::vector<Point>& points) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        Point p;
        if (stream >> p.x >> p.y >> p.z) {
            points.push_back(p);
        }
        else {
            std::cerr << "Línea con formato incorrecto: " << line << std::endl;
        }
    }

    file.close();
    return true;
}

// Función para trazar los puntos
bool handlePlot(const std::string& filename) {
    std::cout << "Presione Enter para continuar y ver el gráfico de los puntos generados..." << std::endl;
    std::cin.get();
    std::cout << "Generando gráfico..." << std::endl;

    std::vector<Point> trajectory_points;
    if (!loadDataFromFile("trajectory_points.txt", trajectory_points)) {
        std::cerr << "Error: No se encontraron puntos en el archivo." << std::endl;
        return false;
    }

    std::vector<double> x, y, z;

    for (const auto& point : trajectory_points) {
        x.push_back(point.x);
        y.push_back(point.y);
        z.push_back(point.z);
    }

    plt::figure();
    // Dibujar todos los puntos
    plt::named_plot("Puntos de la trayectoria", x, y, "b.");
    plt::scatter(x, y, 0.1); // Ajusta el tamaño de los marcadores

    // Incluir la altura en el título
    std::ostringstream title;
    title << "Trayectoria cartesiana generada (Altura = " << z[0] << " mm)";
    plt::title(title.str());

    plt::xlabel("X");
    plt::ylabel("Y");

    // Ajustar el tamaño de los ejes para que tengan la misma escala
    plt::axis("equal");

    // Generar la leyenda
    plt::legend();

    if (!filename.empty()) {
        plt::save(filename);
        std::cout << "Gráfico guardado en: " << filename << std::endl;
    }
    else {
        std::cout << "Mostrando el gráfico " << filename << std::endl;
        plt::show();
    }
    plt::close();

    return true;
}
