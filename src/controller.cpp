#include "socket.h"
#include "LQR.h"
#include "yaml-cpp/yaml.h"
#include "controller.h"


int main() {
    Controller* controller;
    
    // Read the Yaml Config file
    YAML::Node controller_config = YAML::LoadFile("../rsc/controller_setup.yaml");
    const std::string controller_type = controller_config["Controller"]["type"].as<std::string>();       // which controller to use
    const double control_dt = 1 / controller_config["Controller"]["controlRate"].as<double>();           // Rate to compute control actions
    if (controller_type.compare("LQR") == 0) {
        std::vector<double> Qdiag = controller_config["Controller"]["Q"].as<std::vector<double>>();      // Q matrix diagonal
        std::vector<double> Rdiag = controller_config["Controller"]["R"].as<std::vector<double>>();      // R matrix diagonal
        Eigen::Matrix<double, 4, 4> Q;
        Q(0, 0) = Qdiag[0]; Q(1, 1) = Qdiag[1]; Q(2, 2) = Qdiag[2]; Q(3, 3) = Qdiag[3];                  // Construct Q matrix
        Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>(Rdiag[0]);                                       // Construct R matrix
        controller = new LQR(Q, R);
    }

    SocketServer socket(SocketServer::DEFAULT_PORT);

    double RX_state[4] = {0, 0, 0, 0};
    double TX_torque[1] = {0};
    Eigen::Vector4d state;
    double t = 0;
    double t_prev = -1;
    while (true) {
        socket.readSocket(&RX_state, sizeof(RX_state));
        t = RX_state[0];

        // if (t - t_prev >= control_dt) {
            t_prev = t;
            state << RX_state[1], RX_state[2], RX_state[3], RX_state[4];
            TX_torque[0] = controller->getControl(state);
        // }
        socket.writeSocket(&TX_torque, sizeof(TX_torque));
        sleep(control_dt);
    }
    socket.closeSocket();
}
