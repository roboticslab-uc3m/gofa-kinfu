#include "functions.h"
#include "trajectory_generator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <matplotlibcpp.h>
#include <limits>

namespace plt = matplotlibcpp;

// Función para leer datos desde un archivo y devolver vectores de datos
void leerDatos(const std::string& filename, std::vector<std::vector<double>>& datos, std::vector<double>& flags, bool debug) {

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir el archivo: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double a1, a2, a3, a4, a5, a6, flag;
        ss >> a1 >> a2 >> a3 >> a4 >> a5 >> a6 >> flag;

        if (flag == 0) {  // Si el punto es alcanzable
            datos[0].push_back(a1);
            datos[1].push_back(a2);
            datos[2].push_back(a3);
            datos[3].push_back(a4);
            datos[4].push_back(a5);
            datos[5].push_back(a6);
        }
        else {  // Si el punto tiene error, agregar el valor capado
            datos[0].push_back(a1);
            datos[1].push_back(a2);
            datos[2].push_back(a3);
            datos[3].push_back(a4);
            datos[4].push_back(a5);
            datos[5].push_back(a6);
        }
        flags.push_back(flag);
    }

    file.close();

    // Mensaje de depuración
    if (debug) {
        std::cout << "Datos leídos del archivo:" << std::endl;
        for (size_t i = 0; i < datos[0].size(); ++i) {
            std::cout << datos[0][i] << " " << datos[1][i] << " " << datos[2][i] << " "
                << datos[3][i] << " " << datos[4][i] << " " << datos[5][i] << " " << flags[i] << std::endl;
        }
    }

}

// Función para trazar los puntos
bool handlePlotJoints(const InputParameters& input_params, const std::vector<JointLimits>& joint_limits, bool debug, const std::string& filename) {

    std::cout << "Presione Enter para continuar y ver el gráfico de los grados de las articulaciones..." << std::endl;
    std::cin.get();
    std::cout << "Generando gráfico..." << std::endl;

    //Gráficos de KDL
    std::vector<std::vector<double>> datos(6); // Vector de vectores para almacenar datos de cada articulación
    std::vector<double> flags;

    leerDatos("kdl_joint_positions.txt", datos, flags, debug);

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

    // Asumimos que todos los vectores de datos tienen la misma longitud
    int n = datos[0].size();
    if (debug) {
        std::cout << "Número de puntos leídos: " << n << std::endl;
    }

    std::vector<double> tiempo(n); //Vector de tiempo
    for (int i = 0; i < n; ++i) {
        tiempo[i] = i * point_duration;
    }

    if (debug) {
        // Imprimir los datos leídos
        std::cout << "Vector de tiempo:" << std::endl;
        for (const auto& t : tiempo) {
            std::cout << t << " ";
        }
        std::cout << std::endl;

        for (int i = 0; i < 6; ++i) {
            std::cout << "Datos del eje " << i + 1 << ":" << std::endl;
            for (const auto& d : datos[i]) {
                std::cout << d << " ";
            }
            std::cout << std::endl;
        }
    }

    // Añadir NaN en los puntos no alcanzables para las líneas
    std::vector<std::vector<double>> datos_con_nan = datos;
    for (int i = 0; i < n; ++i) {
        if (flags[i] != 0) {  // Si el punto es no alcanzable
            for (int j = 0; j < 6; ++j) {
                datos_con_nan[j][i] = std::numeric_limits<double>::quiet_NaN();  // Insertar NaN
            }
        }
    }

    plt::figure();

    // Dibujar datos alcanzables (líneas, ignorando NaN)
    plt::named_plot("q1", tiempo, datos_con_nan[0], "r-");
    plt::named_plot("q2", tiempo, datos_con_nan[1], "g-");
    plt::named_plot("q3", tiempo, datos_con_nan[2], "b-");
    plt::named_plot("q4", tiempo, datos_con_nan[3], "c-");
    plt::named_plot("q5", tiempo, datos_con_nan[4], "m-");
    plt::named_plot("q6", tiempo, datos_con_nan[5], "y-");

    // Dibujar puntos no alcanzables con scatter
    std::vector<double> tiempo_no_alcanzable;
    std::vector<std::vector<double>> datos_no_alcanzables(6);

    for (int i = 0; i < n; ++i) {
        if (flags[i] != 0) {  // Almacenamos puntos no alcanzables
            tiempo_no_alcanzable.push_back(tiempo[i]);
            for (int j = 0; j < 6; ++j) {
                datos_no_alcanzables[j].push_back(datos[j][i]);
            }
        }
    }

    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[0], 10.0, { {"color", "r"} }); // q1 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[1], 10.0, { {"color", "g"} }); // q2 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[2], 10.0, { {"color", "b"} }); // q3 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[3], 10.0, { {"color", "c"} }); // q4 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[4], 10.0, { {"color", "m"} }); // q5 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[5], 10.0, { {"color", "y"} }); // q6 capado

    // Dibujar los límites articulares usando axhline y diferentes colores
    std::vector<std::string> limit_colors = { "r", "g", "b", "c", "m", "y" };  // Colores para los límites

    for (int i = 0; i < 6; ++i) {
        std::string color = limit_colors[i];
        // Dibujar límites mínimos y máximos
        plt::axhline(joint_limits[i].min, 0, tiempo.back(), { {"color", color}, {"linestyle", ":"}, {"linewidth", "0.5"} });
        plt::axhline(joint_limits[i].max, 0, tiempo.back(), { {"color", color}, {"linestyle", "-."}, {"linewidth", "0.5"} });

        // Convertir los valores de los límites a enteros para no mostrar decimales
        std::string min_label = std::to_string(static_cast<int>(joint_limits[i].min)) + "°";
        std::string max_label = std::to_string(static_cast<int>(joint_limits[i].max)) + "°";

        // Añadir anotaciones para los límites mínimo y máximo, utilizando plt::text para posicionar el texto
        plt::text(tiempo.back() * 1, joint_limits[i].min, min_label);  // Solo posición y texto
        plt::text(tiempo.back() * 1, joint_limits[i].max, max_label);  // Solo posición y texto

        // Calcular el valor mínimo y máximo alcanzado por la articulación
        double min_value = *std::min_element(datos[i].begin(), datos[i].end());
        double max_value = *std::max_element(datos[i].begin(), datos[i].end());

        // Encontrar los tiempos en los que se alcanzan el valor mínimo y máximo
        int min_index = std::distance(datos[i].begin(), std::min_element(datos[i].begin(), datos[i].end()));
        int max_index = std::distance(datos[i].begin(), std::max_element(datos[i].begin(), datos[i].end()));

        // Dibujar cruces en los puntos máximos y mínimos
        plt::scatter(std::vector<double>{tiempo[min_index]}, std::vector<double>{min_value}, 20, { {"color", color}, {"marker", "x"} });
        plt::scatter(std::vector<double>{tiempo[max_index]}, std::vector<double>{max_value}, 20, { {"color", color}, {"marker", "s"} });
    }

    // Crear entradas personalizadas en la leyenda para explicar los diferentes estilos de líneas
    plt::named_plot("Límite máximo", std::vector<double>{}, std::vector<double>{}, "k-.");
    plt::named_plot("Límite mínimo", std::vector<double>{}, std::vector<double>{}, "k:");
    plt::named_plot("Puntos inalcanzables", std::vector<double>{}, std::vector<double>{}, "k.");
    plt::named_plot("Valores máximos", std::vector<double>{}, std::vector<double>{}, "ks");
    plt::named_plot("Valores mínimos", std::vector<double>{}, std::vector<double>{}, "kx");

    plt::xlabel("Tiempo (s)");
    plt::ylabel("Grados");
    plt::title("Evolución temporal de las trayectorias articulares");

    // Crear la leyenda ajustando el tamaño y dividiéndola en dos columnas
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
