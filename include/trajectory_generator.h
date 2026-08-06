#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

// Definición de una estructura para representar un punto en 3D
struct Point {
    double x; // coordenada x
    double y; // coordenada y
    double z; // coordenada z
};

// Definición de la clase TrajectoryGenerator
class TrajectoryGenerator {
public:
    TrajectoryGenerator(const std::vector<double>& centroid, double radius, double height, double inclination, int resolution, double start_position, double end_position, double speed); // Inicializa los parámetros de la circunferencia
    std::vector<Point> generateCircularTrajectory(); // Genera y devuelve un vector de puntos que representan una trayectoria circular en el espacio 3D
    double calculatePointDuration() const; // Calcula la duración para alcanzar cada punto
private: //Almacena los parámetros de entrada
    const std::vector<double>& centroid_;
    double radius_;
    double height_;
    double inclination_;
    int resolution_;
    double start_position_;
    double end_position_;
    //static constexpr double height_per_turn_ = 0.5; // Altura fija entre vueltas, valor constante disponible en tiempo de compilación y que no depende de una instancia concreta de esta clase
    double speed_;
};

#endif // TRAJECTORY_GENERATOR_H
