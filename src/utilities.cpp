#include "utilities.h"

Logger::Logger(std::string file_name) {
    this->file_id.open(file_name);
}

Logger::~Logger() {
    this->file_id.close();
}

void Logger::Write(Eigen::VectorXd data) {
    int n = data.rows();

    for (int i = 0; i < n; i++) {
        this->file_id << data(i) << ",";
    }
    this->file_id << "\n";
}

void Logger::addLabels(std::string labels) {
    this->file_id << labels << std::endl;
}

void PrintMatrix(Eigen::MatrixXd matrix) {
    int n_r = matrix.rows();
    int n_c = matrix.cols();

    for (int i = 0; i < n_r; i++) {
        for (int j = 0; j < n_c; j++) {
            std::cout << matrix(i, j) << "\t";
        }
        std::cout << "\n" << std::endl;
    }
}
