#ifndef utils_h
#define utils_h

#include <Eigen/Dense>

#include "stdio.h"
#include <iostream>
#include <fstream>

class Logger {
    private: 
    std::ofstream file_id;
    
    public: 
    Logger(std::string file_name);
    ~Logger();
    void Write(Eigen::VectorXd data);
    void addLabels(std::string labels);
};

void PrintMatrix(Eigen::MatrixXd matrix);

#endif