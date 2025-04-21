/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef PINHOLE_CAMERA_HPP
#define PINHOLE_CAMERA_HPP

#include "ThinLensCamera.hpp"

class PinholeCamera : public ThinLensCamera {
public:

    PinholeCamera(const std::string &ymlFilename, const std::string &bitmapFilename);

    void render(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &outputImage);

};

std::ostream &operator<<(std::ostream &os, const PinholeCamera &camera);

#endif