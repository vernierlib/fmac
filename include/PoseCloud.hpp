/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef POSE_CLOUD_HPP
#define POSE_CLOUD_HPP

#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "halton_sampler_6.h"

class PoseCloud {
public:

    PoseCloud(int seed, int poseCount, double yawMin, double yawMax, double pitchMin, double pitchMax, double rollMin, double rollMax, double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);

    PoseCloud(const std::string & filename);

    void printPoses();
   
    /// @brief XML/YAML/JSON cloud storage. 
    /// @param filename Name of the file to open or the text string to read the data from. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeMatrix.xml.gz
    void write(const std::string & filename);

    void read(const std::string & filename);

    cv::Vec3d getRVec(int index);

    cv::Vec3d getTVec(int index);

    double getYaw(int index) { return rpy.at<double>(index, 2); }

    double getPitch(int index) { return rpy.at<double>(index, 1); }

    double getRoll(int index) { return rpy.at<double>(index, 0); }

    double getPoseCount() { return poseCount; }

private:
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rpy;

    int seed;
    int poseCount;
    double yawMin;
    double yawMax;
    double pitchMin;
    double pitchMax;
    double rollMin;
    double rollMax;
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double zMin;
    double zMax;
    
    void checkParameters();

    void generate();

    // Calculates rotation matrix given Tait-Bryan angles.
    void taitBryanAnglesToRotationMatrix(double yaw, double pitch, double roll, cv::Mat & result);  

    double scale(double value, double min, double max);

    double scaleWrtDistance(double x, double xMin, double xMax, double z, double zMin);

    friend std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud);
};

std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud);

#endif