/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef THIN_LENS_CAMERA_HPP
#define THIN_LENS_CAMERA_HPP

#include "sobol.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ThinLensCamera {
public:
    // Camera parameters (can be modified before calling the render method)
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    double pixelPitch = -1.0;
    int bitDepth = -1;
    double fNumber = -1.0;
    double focusDistance = -1.0;
    int imageWidth = -1;
    int imageHeight = -1;
    double lightWaveLength = -1.0;
    double backgroundIntensity = 0.5;
    std::string unit = "";

    // Marker parameters (can be modified before calling the render method)
    cv::Mat markerBitmap;
    double markerWidth = -1.0;
    double markerHeight = -1.0;
    double markerOriginX = -1.0;
    double markerOriginY = -1.0;

    // Results
    const int minIntensityValue = 0;
    int maxIntensityValue;
    double minPatternDistance;
    double maxPatternDistance;
    cv::Mat distanceMap;
    cv::Mat distanceToEdgeMap;
    cv::Mat edgeMap;
    cv::Mat confusionMap; // radius of the circle of confusion in pixels
    cv::Mat intensityMap;
    cv::Mat countMap;
    double focalLength;
    double lensRadius;
    double lensToSensorDistance;

    // Other parameters
    double MAX_TILT_ANGLE_IN_DEG = 89.99;
    double MIN_PIXEL_MARGIN = 3.0;
    std::string model = "";
    std::string objectiveLens = "";

    ThinLensCamera(const std::string &ymlFilename, const std::string &bitmapFilename);

    virtual void render(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &outputImage);

    virtual void showMaps();

    /**
     * @brief Read camera paremeters in a XML/YAML/JSON file.
     *
     * @param filename Name of the file to open or the text string to read the data from. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeMatrix.xml.gz
     */
    virtual void readCameraParameters(const std::string &filename);

    /**
     * @brief Writes camera paremeters in a XML/YAML/JSON file.
     *
     * @param filename Name of the file to open or the text string to read the data from. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeMatrix.xml.gz
     */
    virtual void writeCameraParameters(const std::string &filename);

    /**
     * @brief Loads a bitmap of a marker from the specified file. See the OpenCV documentation to know the supported file formats.
     *
     * @param bitmapFilename filename of the bitmap of a marker
     */
    virtual void readMarkerBitmap(const std::string &bitmapFilename);

    inline double circleOfConfusionRadiusInPixels(double objectDistance) {
        return std::fabs(lensRadius * focalLength * (objectDistance - focusDistance) / objectDistance / (focalLength + focusDistance) / pixelPitch);
    }

    inline double airyDiskRadiusInPixels() {
        return 1.22 * lightWaveLength * fNumber / pixelPitch;
    }

protected:
    // Work variables
    Eigen::Matrix4d cTm;
    Eigen::Matrix4d mTc;
    double principalPointX, principalPointY;
    cv::Mat distortionMapX, distortionMapY;
    Eigen::Vector3d markerNormal;
    double markerDistance;
    double markerPixelWidth;
    double markerPixelHeight;
    double focusDistanceOverLensToSensorDistance;
    int colMin, colMax, rowMin, rowMax;
    cv::Mat rotationMatrix;
    double inverseRectificationCoeff;

    virtual void checkParameters();

    virtual void computeFrameTransforms(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    virtual void computeRegionOfInterest(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    virtual void computeRayTracingMetricParameters();

    virtual void computeSharpImageAndDepthMap();

    virtual void computeEdgeMaps();

    virtual void refineImageWithAdaptiveSampling();

    virtual void addDiffractionBlur();

    virtual void quantifyOutputImage(cv::Mat &outputImage);

    inline void concentricMapping(double &ux, double &uy) {
        if (ux != 0.0 || uy != 0.0) {
            double theta, r;
            if (std::abs(ux) > std::abs(uy)) {
                r = ux;
                theta = M_PI_4 * uy / ux;
            } else {
                r = uy;
                theta = M_PI_2 - M_PI_4 * ux / uy;
            }
            ux = r * std::cos(theta);
            uy = r * std::sin(theta);
        }
    }
};

std::ostream &operator<<(std::ostream &os, const ThinLensCamera &camera);

#endif