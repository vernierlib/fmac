/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "MathUtils.hpp"

double gammaCorrection(double lightIntensity) {
    double videoSignal;
    if (lightIntensity <= 0.018) {
        videoSignal = 4.5 * lightIntensity;
    } else {
        videoSignal = 1.099 * std::pow(lightIntensity, 0.45) - 0.099;
    }
    return videoSignal;
}

double airyPattern(double radius, double radialDistance) {
    if (radialDistance < 1e-9) {
        return 1.0;
    } else {
        double x = AIRY_FIRST_ZERO_RADIUS * PI * radialDistance / radius;
        return square(2 * j1(x) / x);
    }
}

void airyKernel(int kernelSize, double radius, cv::Mat & kernel) {
    assert(kernelSize % 2 == 1);
    int center = (kernelSize - 1) / 2;
    kernel.create(cv::Size(kernelSize, kernelSize), CV_64F);
    for (int row = 0; row < kernel.rows; row++) {
        for (int col = 0; col < kernel.cols; col++) {
            double distance = std::hypot(col - center, row - center);
            kernel.at<double>(row, col) = airyPattern(radius, distance);
        }
    }
}

void tophatKernel(int kernelSize, int radius, cv::Mat & kernel) {
    assert(kernelSize % 2 == 1);
    int center = (kernelSize - 1) / 2;
    kernel.create(cv::Size(kernelSize, kernelSize), CV_64F);
    for (int row = 0; row < kernel.rows; row++) {
        for (int col = 0; col < kernel.cols; col++) {
            if (std::abs(col - center) <= radius && std::abs(row - center) <= radius) {
                kernel.at<double>(row, col) = 1.0;
            } else {
                kernel.at<double>(row, col) = 0.0;
            }
        }
    }
}

void discreteAiryKernel(double radiusInPixels, cv::Mat & kernel) {
    int kernelSize = (int) (8 * radiusInPixels);
    if (kernelSize % 2 == 0) {
        kernelSize++;
    }
    int largeKernelSize = kernelSize * KERNEL_SUBDIVISIONS;

    cv::Mat tophat;
    tophatKernel(largeKernelSize, radiusInPixels * KERNEL_SUBDIVISIONS, tophat);

    cv::Mat airy;
    airyKernel(largeKernelSize, radiusInPixels * KERNEL_SUBDIVISIONS, airy);

    cv::Mat largeKernel;
    cv::filter2D(airy, largeKernel, CV_64F, tophat);
    
    cv::resize(largeKernel, kernel, cv::Size(kernelSize, kernelSize), 0, 0, cv::INTER_AREA);
    kernel = kernel / cv::sum(kernel)[0];
}

void taitBryanAnglesToRotationMatrix(double roll, double pitch, double yaw, cv::Mat &result) {
    // Rotation matrix about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

    // Rotation matrix about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));

    // Rotation matrix about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1);

    // Combined rotation matrix
    result = R_z * R_y * R_x;
}

void rotationMatrixToTaitBryanAngles(const cv::Mat & rmat, double & roll, double & pitch, double & yaw) {

    double sy = sqrt(rmat.at<double>(0, 0) * rmat.at<double>(0, 0) + rmat.at<double>(1, 0) * rmat.at<double>(1, 0));

    bool singular = sy < 1e-6;

    if (!singular) {
        roll = atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
        pitch = atan2(-rmat.at<double>(2, 0), sy);
        yaw = atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));
    } else {
        roll = atan2(-rmat.at<double>(1, 2), rmat.at<double>(1, 1));
        pitch = atan2(-rmat.at<double>(2, 0), sy);
        yaw = 0.0;
    }
}

double scale(double value, double min, double max) {
    return min + (max - min) * value;
}

bool isRotationMatrix(const cv::Mat &mat) {
    if (mat.cols!=3 || mat.rows!=3) {
        return false;
    }
    cv::Mat matT;
    transpose(mat, matT);
    cv::Mat shouldBeIdentity = matT * mat;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    return cv::norm(I, shouldBeIdentity) < 1e-6;
}
