#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>

#include "SAE.hpp"
// #include "parameters.hpp"

//---- Constructor Functions ----//
SurfaceOfActiveEvents::SurfaceOfActiveEvents(int height, int width){
    // Resize SAE image to the correct dimensions and set to zero
    SAEimage.resize(height, width);
    SAEimage.setZero();

}

//---- Add Event to SAE ----//
void SurfaceOfActiveEvents::addEvent(Eigen::Vector4d e){
    // int x = e(0);
    // int y = e(1);
    // double t = e(2);
    // int p_signed = (e(3)>0) ? 1 : -1;

    // SAEimage(y, x) = t;
    // t_current = t;

    // std::cout << int(e(1)) << "\t" << int(e(0)) << "\t" << e(2) << std::endl; 
    SAEimage(int(e(1)), int(e(0))) = e(2);
    t_current = e(2);
}
