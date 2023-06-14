#include "socket.h"
#include "stdio.h"
#include <iomanip>
#include "yaml-cpp/yaml.h"
#include "iostream"

#include "mujoco.h"
#include "GLFW/glfw3.h"

#include <Eigen/Dense>

#include "mujoco_interface.h"
#include "utilities.h"

int main() {
    // Read the Yaml Config file
    YAML::Node sim_config = YAML::LoadFile("../rsc/sim_setup.yaml");
    const double mujcoco_visual_update_rate = sim_config["Simulator"]["visual_update_rate"].as<double>();       // Visualization update rate
    const std::string model_file_location = sim_config["Simulator"]["model_file_location"].as<std::string>();   // Model file path
    std::string logger_file_location = sim_config["Simulator"]["logger_file_location"].as<std::string>();       // Logger file path
    const std::vector<double> x0 = sim_config["Simulator"]["x0"].as<std::vector<double>>();                     // Initial state
    const std::vector<double> pert_start = sim_config["Perturbations"]["pert_start"].as<std::vector<double>>();     // Start time of perturbations
    const std::vector<double> pert_end = sim_config["Perturbations"]["pert_end"].as<std::vector<double>>();         // End time of perturbations
    const std::vector<double> pert_force = sim_config["Perturbations"]["pert_force"].as<std::vector<double>>();     // Perturbation force
    const std::vector<double> pert_torque = sim_config["Perturbations"]["pert_torque"].as<std::vector<double>>();   // Perturbations torque

    // Create an instance of the class used to run the mujoco simulation
    MujocoInterface mujoco_interface;
    mujoco_interface.MujocoSetup(const_cast<char*>(model_file_location.c_str()));

    // Create a socket for communicating with controller
    SocketClient socket(SocketClient::DEFAULT_PORT);

    // Define the initial pose
    Vector4d state;
    state << x0[0], x0[1], x0[2], x0[3];

    // Set the initial state
    mujoco_interface.setState(state);
    double t;
    double TX_state[5] = {0, x0[0], x0[1], x0[2], x0[3]};
    double RX_force[1] = {0};
 
    // Initialize Logger
    Logger logger(const_cast<char*>(logger_file_location.c_str()));
    logger.addLabels("t, x, theta, xdot, thetadot, F");

    // Create a vector to store the log data;
    Eigen::VectorXd log_data(2 + state.rows());
    // The simulation loop
    while(!glfwWindowShouldClose(mujoco_interface.window)) {
        // Get the current simulation time
        mjtNum simstart = MJ_DATA_PTR->time;

        // This loop ensures that the mujoco visuals updates at the rate specified by mujcoco_visual_update_rate
        while(MJ_DATA_PTR->time - simstart < 1.0 / mujcoco_visual_update_rate) {      
            // get states, time
            state = mujoco_interface.getState();
            t = MJ_DATA_PTR->time;            

            // Send data to controller, recieve control
            TX_state[0] = t;
            TX_state[1] = state(0);
            TX_state[2] = state(1);
            TX_state[3] = state(2);
            TX_state[4] = state(3);
            socket.writeSocket(&TX_state, sizeof(TX_state));
            socket.readSocket(&RX_force, sizeof(RX_force));

            // Write data to logger
            log_data << t, state(0), state(1), state(2), state(3), RX_force[0];
            logger.Write(log_data);

            // Set forces
            MJ_DATA_PTR->qfrc_applied[0] = 0;
            MJ_DATA_PTR->qfrc_applied[1] = 0;
            for (int ii = 0; ii < pert_start.size(); ii++) {
                if (t < pert_end[ii] && t >= pert_start[ii]) {
                    MJ_DATA_PTR->qfrc_applied[0] += pert_force[ii];
                    MJ_DATA_PTR->qfrc_applied[1] += pert_torque[ii];
                }
            }
            // And control action
            MJ_DATA_PTR->ctrl[0] = RX_force[0];

            // Propagate the dynamics
            mj_step(MJ_MODEL_PTR, MJ_DATA_PTR);       
        }

        // Update the visuals
        mujoco_interface.UpdateScene();
    }

    // Close the mujoco simulation
    mujoco_interface.MujocoShutdown();
    socket.closeSocket();

    return 0;
}
