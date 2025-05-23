/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef THIN_LENS_CAMERA_HPP
#define THIN_LENS_CAMERA_HPP

#include "MathUtils.hpp"
#include "sobol.h"

#define MAX_TILT_ANGLE_IN_DEG 89.99
#define MIN_PIXEL_MARGIN  3.0
#define MIN_MARKER_BITMAP_RESOLUTION 1000

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
    std::string brand = "";

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

    virtual double circleOfConfusionRadiusInPixels(double objectDistance);

    virtual double airyDiskRadiusInPixels() const;
    
    virtual double airyDiskRadius() const;

    /** Returns the angle of view measured diagonally. */
    virtual double angleOfViewInDeg() const;

    /** Returns the distance between the nearest and the farthest planes that are in acceptably sharp focus. */
    virtual double depthOfField() const;

    /** Returns the distance from sharp foreground */
    virtual double nearDepthOfFieldLimit() const;

    /** Returns the distance from sharp background */
    virtual double farDepthOfFieldLimit() const;

    /** Returns the focus distance that maximize the depth of field */
    virtual double hyperfocalDistance() const;

    /** Returns the coordinates of the four marker corners in the image (pixels). */
    std::vector<cv::Point2d> markerCorners(const cv::Vec3d &rvec, const cv::Vec3d &tvec) const;

    virtual std::string toString() const;

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

    virtual void applyGammaCorrection();

    virtual void quantifyOutputImage(cv::Mat &outputImage);
    
};

std::ostream &operator<<(std::ostream &os, const ThinLensCamera &camera);

#endif