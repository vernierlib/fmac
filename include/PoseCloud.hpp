/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef POSE_CLOUD_HPP
#define POSE_CLOUD_HPP

#include "MathUtils.hpp"
#include "halton_sampler_6.h"
#include "PoseBox.hpp"
#include "ThinLensCamera.hpp"

class PoseCloud {
public:

    /// @brief Creates a cloud of randomly generated poses uniformly spread inside a rectangular box.
    /// @param seed Seed of the Halton sampler
    /// @param poseCount Number of pose to generate
    /// @param box boundaries of the poses
    PoseCloud(int seed, int poseCount, PoseBox box);

    /// @brief Creates a cloud of randomly generated poses uniformly spread inside the field of view of a camera.
    /// @param seed Seed of the Halton sampler
    /// @param poseCount Number of pose to generate
    /// @param box Boundaries of the poses (xMin, xMax, yMin, yMax are not used)
    /// @param camera Camera used to define the field of view
    PoseCloud(int seed, int poseCount, PoseBox box, const ThinLensCamera & camera);

    /// @brief Creates a cloud of poses from a file.
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.xml.gz
    PoseCloud(const std::string & filename);

    /// @brief Saves a cloud of poses in a file. 
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.gz
    void write(const std::string & filename);

    /// @brief Loads a cloud of poses from a file.
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.xml.gz
    void read(const std::string & filename);

    /// Returns the number of poses of the cloud.

    double getPoseCount() {
        return poseCount;
    }

    /// @brief Returns the translation vector at a given index.
    cv::Vec3d getTVec(int index);

    /// @brief Returns the rotation vector at a given index (see Rodrigues).
    cv::Vec3d getRVec(int index);

    /// @brief Returns the yaw angle Rz at a given index.

    double getYaw(int index) {
        return rpy.at<double>(index, 2);
    }

    /// @brief Returns the pitch angle Ry at a given index.

    double getPitch(int index) {
        return rpy.at<double>(index, 1);
    }

    /// @brief Returns the roll angle Rx at a given index.

    double getRoll(int index) {
        return rpy.at<double>(index, 0);
    }

    /// @brief Sets the translation vector at a given index.
    void setTVec(int index, const cv::Mat & tvec);

    /// @brief Sets the translation vector at a given index.
    void setTVec(int index, double x, double y, double z);

    /// @brief Sets the rotation vector at a given index.
    void setTaitBryanAngles(int index, double roll, double pitch, double yaw);

    /// @brief Sets the rotation vector at a given index.
    void setRVec(int index, const cv::Mat & rvec);

    /// @brief Sets the vector of the four marker corners.
    void setMarkerCorners(int index, const std::vector<cv::Point2d> & corners);

    /// @brief Sets the vector of the four marker corners.
    void setMarkerCorners(int index, const std::vector<cv::Point2f> & corners);

    /// @brief Draws all the poses of the cloud in an image using a given camera.    
    void draw(cv::Mat & image, const ThinLensCamera & camera);

private:
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rpy;
    cv::Mat markerCorners;
    int seed;
    int poseCount;
    PoseBox box;

    void checkParameters();

    friend std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud);
};

/// @brief Prints all the poses of the cloud in the output stream.
std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud);

#endif