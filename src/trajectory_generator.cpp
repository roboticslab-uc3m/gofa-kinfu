// La lógica para implementar la trayectoria circular
#define _USE_MATH_DEFINES
#include <cmath>
#include "trajectory_generator.h"
#include <iostream>


// Inicializa los miembros de la clase, la inclinación se convierte de grados a radianes
TrajectoryGenerator::TrajectoryGenerator(const std::vector<double>& centroid, double radius, double height, double inclination, int resolution, double start_position, double end_position, double speed)
    : centroid_(centroid), radius_(radius), height_(height), inclination_(inclination* M_PI / 180.0), resolution_(resolution), start_position_(start_position* M_PI / 180.0), end_position_(end_position* M_PI / 180.0), speed_(speed) {}

std::vector<Point> TrajectoryGenerator::generateCircularTrajectory() {
    std::vector<Point> trajectory; // Inicializa un vector para almacenar los puntos generados
    double angle_increment = (end_position_ - start_position_) / resolution_; // Incremento del ángulo, para generar puntos equidistantes a lo largo de la circunferencia. Calcula el ángulo entre cada punto consecutivo de la circunferencia

    //// Calcula el ángulo inicial en función de la posición de partida
    //double start_angle = atan2(start_position_[1] - centroid_[1], start_position_[0] - centroid_[0]);

    // Generar los puntos de la trayectoria entre start_position_ y end_position_
        for (int i = 0; i < resolution_; ++i) { // Bucle que genera los puntos de la circunferencia, itera de 0 a resolution-1
            // Calcula t (ángulo actual del punto que está calculando) y las coordenadas x, y, z de cada punto de la circunferencia
            double t = start_position_ + i * angle_increment;
            double x = radius_ * cos(t);
            double y = radius_ * sin(t);
            double z = height_; // Altura constante

            // Ajustar al centro y altura
            Point p;
            p.x = centroid_[0] + x;
            p.y = centroid_[1] + y;
            p.z = centroid_[2] + z;

            trajectory.push_back(p); // Añade el punto generado en el vector de trayectoria
        }

    return trajectory;
}

double TrajectoryGenerator::calculatePointDuration() const {
    double circumference = (end_position_ - start_position_) * radius_; // Calcula la circunferencia completa
    double total_duration = circumference / speed_; // Calcula el tiempo total para completar una vuelta
    return total_duration / resolution_; // Divide el tiempo total por el número de puntos para obtener la duración por punto
}
