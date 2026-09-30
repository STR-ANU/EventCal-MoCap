#ifndef SAE_HPP
#define SAE_HPP

#include <eigen3/Eigen/Dense>
// #include "parameters.hpp"

class SurfaceOfActiveEvents
{
public:
    //------------------------------------------//
    //-----        PUBLIC FUNCTIONS        -----//
    //------------------------------------------//
    // Constructor functions
    SurfaceOfActiveEvents(int height, int width);
    // Add event to SAE
    void addEvent(Eigen::Vector4d e);
    // Access the image
    inline const Eigen::MatrixXd& getImage() const { return SAEimage; };

    //------------------------------------------//
    //-----        PUBLIC VARIABLES        -----//
    //------------------------------------------//
    double t_current = 0;

private:

    //------------------------------------------//
    //-----        PRIVATE VARIABLES       -----//
    //------------------------------------------//
    Eigen::MatrixXd SAEimage;
};

#endif