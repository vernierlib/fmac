/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <math.h>
#include "j1.h" /* first order Bessel function */

#define PI 3.14159265358979323846264338327950288
#define PI_OVER_2 1.57079632679489661923132169163975144
#define PI_OVER_4 0.785398163397448309615660845819875721
#define AIRY_FIRST_ZERO_RADIUS 1.219669891266504
#define KERNEL_SUBDIVISIONS 65

inline double square(double x) {
    return x*x;
}

/// Applies the Rec. 709 transfer function 
double gammaCorrection(double lightIntensity);

double airyPattern(double radius, double radialDistance);

void airyKernel(int kernelSize, double radius, cv::Mat & kernel);

void tophatKernel(int kernelSize, int radius, cv::Mat & kernel);

void discreteAiryKernel(double radiusInPixels, cv::Mat & kernel);

inline void concentricMapping(double &ux, double &uy) {
    if (ux != 0.0 || uy != 0.0) {
        double theta, r;
        if (std::abs(ux) > std::abs(uy)) {
            r = ux;
            theta = PI_OVER_4 * uy / ux;
        } else {
            r = uy;
            theta = PI_OVER_2 - PI_OVER_4 * ux / uy;
        }
        ux = r * std::cos(theta);
        uy = r * std::sin(theta);
    }
}

/// Calculates rotation matrix given Tait-Bryan angles.
void taitBryanAnglesToRotationMatrix(double roll, double pitch, double yaw, cv::Mat & result);

/// Calculates Tait-Bryan angle given a rotation matrix.
void rotationMatrixToTaitBryanAngles(const cv::Mat & rmat, double & roll, double & pitch, double & yaw);

/// Scales a value from [0;1[ to [min;max[
double scale(double value, double min, double max);

/// Returns true if mat is a rotation matrix 
bool isRotationMatrix(const cv::Mat &mat);

#endif