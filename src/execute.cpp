#include <fstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include "trajectory_generator.h" // Incluye la definición de la clase TrajectoryGenerator que genera la trayectoria circular
#include "parser.h"
#include "functions.h"
#include <abb_libegm/egm_trajectory_interface.h> // Incluye la interfaz de trayectorias EGM para la comunicación con el robot ABB
//#include <abb_libegm/egm_controller_interface.h> // Para integrar la lectura de posiciones reales del robot
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <SceneReconstructionIDL.h>

bool handleExecute(const std::string& jointsFile, const InputParameters& input_params) {
    std::cout << "Presione Enter para continuar y mover el robot al primer punto de la trayectoria..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Iniciar YARP server
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::cerr << "No se pudo conectar al servidor de YARP\n";

    }

    // Abrir un puerto y conectarse a servicio externo
    yarp::os::RpcClient rpc;

    if (!rpc.open("/trajectory_generator_node/rpc:c") || !yarp::os::Network::connect(rpc.getName(), "/sceneReconstruction/rpc:s"))
    {
        std::cerr<< "No se pudo establecer la conexión con el servidor de recostrucción\n";

    }

    roboticslab::SceneReconstructionIDL sceneReconstruction;
    sceneReconstruction.yarp().attachAsClient(rpc);

    // Fase 1: Mover el robot al primer punto
    std::ifstream jointFile(jointsFile);
    if (!jointFile.is_open()) {
        std::cerr << "No se pudo abrir el archivo con los puntos de la trayectoria.\n";
        return false;
    }

    std::vector<double> first_joint_pos(6);
    int reachable;

    // Leer el primer punto del archivo
    std::string line;
    if (std::getline(jointFile, line)) {
        std::istringstream stream(line);
        if (stream >> first_joint_pos[0] >> first_joint_pos[1] >> first_joint_pos[2] >> first_joint_pos[3] >> first_joint_pos[4] >> first_joint_pos[5] >> reachable) {
            if (reachable == 1) {
                std::cerr << "El primer punto es inalcanzable o tiene un formato incorrecto.\n";
                return false;
            }
        }
    }
    else {
        std::cerr << "El archivo está vacío o no tiene el formato esperado.\n";
        return false;
    }

    // Comunicación EGM para mover al primer punto
    // Componentes de Boost para manejar la comunicación asíncrona
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Configura la interfaz EGM para comunicarse con el robot
    abb::egm::EGMTrajectoryInterface egm_interface(io_service, 6512);// Asignar  aquí el número de puerto correcto

    // Verifica que la interfaz de EGM se haya inicializado correctamente
    if (!egm_interface.isInitialized()) {
        std::cerr << "La inicialización de la interfaz EGM ha fallado\n";
        return false;
    }
    else {
        std::cout << "La interfaz EGM se ha inicializado correctamente\n";
    };

    // Crea un hilo para ejecutar io_service
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // Verifica que el robot esté conectado antes de enviar la trayectoria
    while (!egm_interface.isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));// Si no está conectado, el hilo descansa durante 500 ms antes de volver a verificar
    }

    // Mover al primer punto
    abb::egm::wrapper::trajectory::TrajectoryGoal initial_trajectory;
    abb::egm::wrapper::trajectory::PointGoal* initial_point = initial_trajectory.add_points();

    for (size_t i = 0; i < first_joint_pos.size(); ++i) {
        initial_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(first_joint_pos[i]);
    }
    initial_point->set_duration(5.0); // Asignar una duración para alcanzar el primer punto

    std::cout << "Moviendo al primer punto...\n";
    egm_interface.addTrajectory(initial_trajectory);

    // Espera a que la ejecución de la trayectoria termine
    abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
    bool wait = true;
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = execution_progress.goal_active();// goal_active() devuelve true si la trayectoria aún está en curso, y false si ha terminado. Esto actualiza la variable wait, permitiendo que el bucle finalice cuando la ejecución de la trayectoria haya terminado
        }
    }
    std::cout << "Robot alcanzó el primer punto.\n";

    // Leer las posiciones articulares
    std::vector<std::vector<double>> joint_positions;
    std::vector<double> durations;
    bool all_points_reachable = true;
    int line_number = 0;

    // Generar la trayectoria y calcular la duración de cada punto
    TrajectoryGenerator generator(input_params.centroid,
        input_params.radius,
        input_params.height,
        input_params.inclination,
        input_params.resolution,
        input_params.start_position,
        input_params.end_position,
        input_params.speed);

    double point_duration = generator.calculatePointDuration();

    while (std::getline(jointFile, line)) {
        std::istringstream stream(line);
        std::vector<double> joint_pos(6);

        if (stream >> joint_pos[0] >> joint_pos[1] >> joint_pos[2] >> joint_pos[3] >> joint_pos[4] >> joint_pos[5] >> reachable) {
            if (line_number > 0) {  // Iniciar desde el segundo punto
                if (reachable == 1) {
                    all_points_reachable = false;
                    std::cerr << "Punto inalcanzable encontrado en la línea " << line_number + 1 << ": ("
                        << joint_pos[0] << ", " << joint_pos[1] << ", " << joint_pos[2] << ", "
                        << joint_pos[3] << ", " << joint_pos[4] << ", " << joint_pos[5] << ")\n";
                    break;
                }
                joint_positions.push_back(joint_pos);
                durations.push_back(point_duration);
            }
        }
        else {
            std::cerr << "Formato de línea incorrecto: " << line << std::endl;
        }
        line_number++;
    }
    jointFile.close();

    if (!all_points_reachable) {
        std::cerr << "Hay puntos inalcanzables en la trayectoria. La trayectoria no se puede enviar." << std::endl;
        return false;
    }

    std::cout << "Todos los puntos generados son alcanzables. La trayectoria se puede enviar." << std::endl;

    // Crear un hilo para ejecutar io_service
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    // Verifica que el robot esté conectado antes de enviar la trayectoria
    while (!egm_interface.isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Crear la trayectoria para enviarla al robot
    abb::egm::wrapper::trajectory::TrajectoryGoal trajectory;

    for (size_t i = 0; i < joint_positions.size(); ++i) {
        abb::egm::wrapper::trajectory::PointGoal* traj_point = trajectory.add_points();

        for (unsigned int j = 0; j < joint_positions[i].size(); ++j) {
            traj_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(joint_positions[i][j]);
        }

        traj_point->set_duration(durations[i]);
    }

    std::cout << "Enviando trayectoria circular...\n";

    egm_interface.addTrajectory(trajectory);

    sceneReconstruction.resume();

    wait = true;
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = execution_progress.goal_active();
        }
    }

    std::cout << "Trayectoria finalizada\n";
    sceneReconstruction.pause();

    // Apagado limpio del programa
    io_service.stop();
    thread_group.join_all();


    ////Guardar posiciones articulares durante la ejecución (metodo real) NO NOS FUNCIONÓ
    //abb::egm::EGMControllerInterface egm_ctrl_interface(io_service, 6512); // Puerto para la lectura de posiciones

    ////Lectura de posiciones reales
    //std::ofstream joint_positions_file("joint_positions.txt");
    //if (!joint_positions_file.is_open()) {
    //    std::cerr << "No se pudo abrir el archivo para escribir las posiciones articulares.\n";
    //    return 1;
    //}

    //abb::egm::wrapper::Input input; //Almacena datos recibidos del robot
    //while (true) { //Guarda posiciones articulares del robot
    //    if (egm_ctrl_interface.waitForMessage(500)) {
    //        egm_ctrl_interface.read(&input);
    //        int sequence_number = input.header().sequence_number();

    //        for (int i = 0; i < input.feedback().robot().joints().position().values_size(); ++i) {
    //            joint_positions_file << input.feedback().robot().joints().position().values(i) << " ";
    //        }
    //        joint_positions_file << "\n";

    //        std::cout << "Posiciones articulares: ";
    //        for (int i = 0; i < input.feedback().robot().joints().position().values_size(); ++i) {
    //            std::cout << input.feedback().robot().joints().position().values(i) << " ";
    //        }
    //        std::cout << "\n";
    //    }
    //}

    //joint_positions_file.close();

return true;
}
