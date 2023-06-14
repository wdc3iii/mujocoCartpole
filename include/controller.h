#ifndef controller_h
#define controller_h
#include <Eigen/Dense>

class Controller {
    public:
    virtual double getControl(Eigen::Vector4d state) = 0;
};

#endif