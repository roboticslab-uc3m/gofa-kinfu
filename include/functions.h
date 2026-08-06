#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "trajectory_generator.h"
#include "parser.h"
#include <string>
#include <vector>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Función para manejar la generación de la trayectoria
std::vector <Point> handleGenerate(const InputParameters& input_params, bool debug);

// Función para leer los parametros de entrada del teclado si el usuario lo solicita
void readParameters(InputParameters& input_params);

// Función para manejar el ploteo de datos de la trayectoria
bool handlePlot(const std::string& filename = "");
bool loadDataFromFile(const std::string& filename, std::vector<Point>& points);

// Función para manejar la resolución de la cinemática inversa con KDL
bool handleSolveIK(const std::string& robotParams, const std::string& pointsFile, const std::string& outputFile, const InputParameters& input_params, const std::vector<JointLimits>& joint_limits, bool debug);

// Función para manejar el ploteo de las articulaciones
bool handlePlotJoints(const InputParameters& input_params, const std::vector<JointLimits>& joint_limits, bool debug, const std::string& filename = "");
void leerDatos(const std::string& filename, std::vector<std::vector<double>>& datos, std::vector<double>& flags, bool debug);
//void verificarLimitesArticulares(std::vector<std::vector<double>>& datos, const std::vector<JointLimits>& joint_limits);

// Función para manejar la ejecución del movimiento del robot
bool handleExecute(const std::string& jointsFile, const InputParameters& input_params);

std::vector<Point> generateTrajectory(const InputParameters& input_params);

#endif // FUNCTIONS_H
