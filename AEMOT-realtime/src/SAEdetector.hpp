#ifndef SAEDETECTOR_HPP
#define SAEDETECTOR_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>
#include "SAE.hpp" 
// #include "parameters.hpp"

class SAEdetector
{
public:
    // Constructor and initialisation
    // SAEdetector(const Parameters& params);
    SAEdetector(int height, int width, int ksize, double alpha, double min_contributions, double min_active_pixels, double detection_threshold, double dt_detection_threshold, double recency_window);

    // SAEdetector(Parameters &params); 
    SAEdetector();
    ~SAEdetector();
    
    // Functions
    void addEvent(Eigen::Vector4d e);
    int performDetection(Eigen::Vector4d e);
    int performDetection_dt(Eigen::Vector4d e);


    // Access the image
    const Eigen::MatrixXd& getImage() const;

    void getLastDirection(int col, int row, double& lx_out, double& ly_out) const {
        lx_out = lx(row, col);
        ly_out = ly(row, col);
    }

    //------------------------------------------//
    //-----        PUBLIC VARIABLES        -----//
    //------------------------------------------//
    double m_t_current = 0;

private:
    //------------------------------------------//
    //-----        PRIVATE FUNCTIONS       -----//
    //------------------------------------------//
    Eigen::MatrixXd getTimePatch(Eigen::Vector4d e);


    // Internal functions
    void getPatches(Eigen::Vector4d e);

    void computeDirectionRegression(Eigen::Vector4d e);
    int compareDirectionRegression(Eigen::Vector4d e);

    //------------------------------------------//
    //-----       PRIVATE VARIABLES        -----//
    //------------------------------------------//
    SurfaceOfActiveEvents* SAE;

    //----- Parameters -----//
    // Set some default values
    double m_alpha = 1e-5;
    double m_min_active_pixels = 5;
    double m_min_contributions = 2;
    double m_detection_threshold = 0.7;
    int m_ksize;
    double m_dt_detection_threshold = 0;
    double m_recency_window = 0.010; // in seconds (0.010 = 10ms)

    //----- Flags -----//
    bool m_f_patch_status = false;
    bool m_f_regression_status = false;

    Eigen::Vector2d smallestEigenvector2x2(const Eigen::Matrix2d& M) const;

    int m_width, m_height;
    Eigen::MatrixXd initialised_events;
    Eigen::MatrixXd init_events_patch, t_patch;

    Eigen::MatrixXd initialised_directions, init_directions_patch;
    Eigen::MatrixXd lx, ly;
    Eigen::MatrixXd lx_patch, ly_patch;

    Eigen::MatrixXd weight_patch; // cached exp(-2*alpha*(t_current - t_patch)), computed once per event in getPatches()
    
    // Set up some patches that we will need each iteration
    // Eigen::VectorXd b;
    Eigen::VectorXd patch_offset;
    std::vector<std::pair<int, int>> patch_points;
    Eigen::Vector<double, 3> no_detection = Eigen::Vector<double, 3> ::Constant(-1);
    
    Eigen::MatrixXd A_scratch; // (ksize*ksize) x 2, preallocated in ctor
    Eigen::VectorXd W_scratch; // (ksize*ksize), preallocated in ctor
};

#endif