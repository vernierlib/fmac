/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "PinholeCamera.hpp"

PinholeCamera::PinholeCamera(const std::string &ymlFilename, const std::string &bitmapFilename)
: ThinLensCamera(ymlFilename, bitmapFilename) {
}

void PinholeCamera::render(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &outputImage) {
    checkParameters();
    computeRayTracingMetricParameters();
    computeFrameTransforms(rvec, tvec);
    computeRegionOfInterest(rvec, tvec);
    computeSharpImageAndDepthMap();
    //computeEdgeMaps();
    //refineImageWithAdaptiveSampling();
    addDiffractionBlur();
    quantifyOutputImage(outputImage);
}

std::ostream &operator<<(std::ostream &os, const PinholeCamera &camera) {
    os << "Pinhole camera: " << camera.model << " " << camera.imageWidth << "x" << camera.imageHeight << " " << camera.bitDepth << "bits with objective lens " << camera.focalLength << camera.unit << " f/" << camera.fNumber << " fd:" << camera.focusDistance << camera.unit;
    return os;
}

