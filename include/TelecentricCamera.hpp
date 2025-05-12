/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef TELECENTRIC_CAMERA_HPP
#define TELECENTRIC_CAMERA_HPP

#include "ThinLensCamera.hpp"

class TelecentricCamera : public ThinLensCamera {
public:
    
    TelecentricCamera(const std::string &ymlFilename, const std::string &bitmapFilename);

//    inline double circleOfConfusionRadiusInPixels(double objectDistance) {
//        return std::fabs(lensRadius * focalLength * (objectDistance - focusDistance) / objectDistance / (focalLength + focusDistance) / pixelPitch);
//    }
//
//    inline double airyDiskRadiusInPixels() {
//        return 1.22 * lightWaveLength * fNumber / pixelPitch;
//    }
    
    std::vector<cv::Point2d> markerCorners(const cv::Vec3d &rvec, const cv::Vec3d &tvec) const;

private:
    
    void computeRegionOfInterest(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    void computeSharpImageAndDepthMap();

    void refineImageWithAdaptiveSampling();

};

std::ostream &operator<<(std::ostream &os, const TelecentricCamera &camera);

#endif