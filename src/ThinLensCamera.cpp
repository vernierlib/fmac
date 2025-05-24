/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "ThinLensCamera.hpp"
#include "sobol.h"

ThinLensCamera::ThinLensCamera(const std::string &ymlFilename, const std::string &bitmapFilename) {
    readCameraParameters(ymlFilename);
    readMarkerBitmap(bitmapFilename);
    checkParameters();
    computeRayTracingMetricParameters();
}

void ThinLensCamera::render(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &outputImage) {
    checkParameters();
    computeRayTracingMetricParameters();
    computeFrameTransforms(rvec, tvec);
    computeRegionOfInterest(rvec, tvec);
    computeSharpImageAndDepthMap();
    computeEdgeMaps();
    refineImageWithAdaptiveSampling();
    addDiffractionBlur();
    applyGammaCorrection();
    quantifyOutputImage(outputImage);
}

void ThinLensCamera::showMaps() {
    cv::Mat image0;
    countMap.convertTo(image0, CV_32F, 1);
    cv::normalize(image0, image0, 1.0, 0, cv::NORM_MINMAX);
    cv::imshow("Count map", image0);
    cv::moveWindow("Count map", 50, 50);

    cv::Mat image1;
    distanceToEdgeMap.convertTo(image1, CV_8U, 1);
    cv::imshow("Distance-to-egde map", image1);
    cv::moveWindow("Distance-to-egde map", 50, 500);

    cv::Mat image2;
    cv::normalize(distanceMap, image2, 1.0, 0, cv::NORM_MINMAX);
    cv::imshow("Depth map", image2);
    cv::moveWindow("Depth map", 500, 50);

    cv::Mat image3;
    cv::normalize(confusionMap, image3, 1.0, 0, cv::NORM_MINMAX);
    cv::imshow("Confusion map", image3);
    cv::moveWindow("Confusion map", 500, 500);
}

void ThinLensCamera::readCameraParameters(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not find or read the camera parameter file: " + filename);
    }
    file["image_width"] >> imageWidth;
    file["image_height"] >> imageHeight;
    file["camera_matrix"] >> cameraMatrix;
    file["distortion_coefficients"] >> distortionCoefficients;
    file["bit_depth"] >> bitDepth;
    file["focus_distance"] >> focusDistance;
    file["pixel_pitch"] >> pixelPitch;
    file["f_number"] >> fNumber;
    file["marker_width"] >> markerWidth;
    file["marker_height"] >> markerHeight;
    file["marker_origin_x"] >> markerOriginX;
    file["marker_origin_y"] >> markerOriginY;
    file["light_wavelength"] >> lightWaveLength;
    file["background_intensity"] >> backgroundIntensity;
    file["unit"] >> unit;
    cv::FileNode cameraBrandNode = file["brand"];
    if (cameraBrandNode.empty()) {
        brand = "Unknown camera";
    } else {
        file["brand"] >> brand;
    }
    file.release();
}

void ThinLensCamera::checkParameters() {
    assert(imageWidth > 0);
    assert(imageHeight > 0);
    assert((cameraMatrix.cols == 3) && (cameraMatrix.rows == 3));
    assert(bitDepth > 0);
    assert(focusDistance > 0.0);
    assert(pixelPitch > 0.0);
    assert(fNumber > 0.0);
    assert(markerWidth > 0.0);
    assert(markerHeight > 0.0);
    assert((markerOriginX >= 0.0) && (markerOriginX <= markerWidth));
    assert((markerOriginY >= 0.0) && (markerOriginX <= markerHeight));
    assert(lightWaveLength > 0.0);
    assert((markerBitmap.cols > 0) && (markerBitmap.rows > 0));
    assert((backgroundIntensity >= 0.0) && (backgroundIntensity <= 1.0));
}

void ThinLensCamera::writeCameraParameters(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not write the camera parameter file: " + filename);
    }
    file << "image_width" << imageWidth;
    file << "image_height" << imageHeight;
    file << "camera_matrix" << cameraMatrix;
    file << "distortion_coefficients" << distortionCoefficients;
    file << "bit_depth" << bitDepth;
    file << "focus_distance" << focusDistance;
    file << "pixel_pitch" << pixelPitch;
    file << "f_number" << fNumber;
    file << "marker_width" << markerWidth;
    file << "marker_height" << markerHeight;
    file << "marker_origin_x" << markerOriginX;
    file << "marker_origin_y" << markerOriginY;
    file << "light_wavelength" << lightWaveLength;
    file << "background_intensity" << backgroundIntensity;
    file << "unit" << unit;
    file << "brand" << brand;
    file.release();
}

void ThinLensCamera::readMarkerBitmap(const std::string &bitmapFilename) {
    markerBitmap = cv::imread(bitmapFilename, cv::IMREAD_GRAYSCALE);
    if (markerBitmap.empty()) {
        throw std::runtime_error("Could not find or read the marker bitmap: " + bitmapFilename);
    }
    markerBitmap.convertTo(markerBitmap, CV_64F, 1);
    cv::normalize(markerBitmap, markerBitmap, 1.0, 0, cv::NORM_MINMAX);
    int minRowCol = std::min(markerBitmap.rows, markerBitmap.cols);
    if (minRowCol < MIN_MARKER_BITMAP_RESOLUTION) {
        int factor = (MIN_MARKER_BITMAP_RESOLUTION / minRowCol) + 1;
        cv::resize(markerBitmap, markerBitmap, cv::Size(factor * markerBitmap.cols, factor * markerBitmap.rows), 0, 0, cv::INTER_NEAREST);
    }
}

void ThinLensCamera::computeRayTracingMetricParameters() {

    maxIntensityValue = 1 << bitDepth;

    principalPointX = cameraMatrix.at<double>(0, 2);
    principalPointY = cameraMatrix.at<double>(1, 2);

    focalLength = 0.5 * pixelPitch * (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1));
    lensRadius = 0.5 * focalLength / fNumber;
    lensToSensorDistance = focalLength * focusDistance / (focusDistance - focalLength);

    focusDistanceOverLensToSensorDistance = focusDistance / lensToSensorDistance;

    markerPixelWidth = markerWidth / markerBitmap.cols;
    markerPixelHeight = markerHeight / markerBitmap.rows;
}

void ThinLensCamera::computeFrameTransforms(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    cTm.setIdentity();
    cTm(0, 3) = tvec(0);
    cTm(1, 3) = tvec(1);
    cTm(2, 3) = tvec(2);

    cv::Rodrigues(rvec, rotationMatrix);
    cTm(0, 0) = rotationMatrix.at<double>(0, 0);
    cTm(1, 0) = rotationMatrix.at<double>(1, 0);
    cTm(2, 0) = rotationMatrix.at<double>(2, 0);
    cTm(0, 1) = rotationMatrix.at<double>(0, 1);
    cTm(1, 1) = rotationMatrix.at<double>(1, 1);
    cTm(2, 1) = rotationMatrix.at<double>(2, 1);
    cTm(0, 2) = rotationMatrix.at<double>(0, 2);
    cTm(1, 2) = rotationMatrix.at<double>(1, 2);
    cTm(2, 2) = rotationMatrix.at<double>(2, 2);
    mTc = cTm.inverse();

    markerNormal = Eigen::Vector3d(mTc(2, 0), mTc(2, 1), mTc(2, 2));
    markerDistance = mTc(2, 3);

    Eigen::Vector3d rayDirection(0.0, 0.0, 1.0);
    double cosAngle = markerNormal.dot(rayDirection);
    if (cosAngle < cos(MAX_TILT_ANGLE_IN_DEG * PI / 180.0)) {
        throw std::runtime_error("The marker is not visible (tilt angle too large)");
    }

    cv::initInverseRectificationMap(cameraMatrix, distortionCoefficients, cv::Mat(), cameraMatrix, cv::Size(imageWidth, imageHeight), CV_32FC1, distortionMapX, distortionMapY);
    inverseRectificationCoeff = focusDistance / (focusDistance - focalLength);
}

std::vector<cv::Point2d> ThinLensCamera::markerCorners(const cv::Vec3d &rvec, const cv::Vec3d &tvec) const {
    std::vector<cv::Point3d> markerPoints(4);
    markerPoints[0] = cv::Point3d(0.0 - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerPoints[1] = cv::Point3d(markerWidth - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerPoints[2] = cv::Point3d(markerWidth - markerOriginX, markerHeight - markerOriginY, 0.0);
    markerPoints[3] = cv::Point3d(0.0 - markerOriginX, markerHeight - markerOriginY, 0.0);

    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(markerPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    return imagePoints;
}

void ThinLensCamera::computeRegionOfInterest(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    std::vector<cv::Point3d> markerPoints(4);
    markerPoints[0] = cv::Point3d(0.0 - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerPoints[1] = cv::Point3d(markerWidth - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerPoints[2] = cv::Point3d(markerWidth - markerOriginX, markerHeight - markerOriginY, 0.0);
    markerPoints[3] = cv::Point3d(0.0 - markerOriginX, markerHeight - markerOriginY, 0.0);

    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(markerPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    colMin = std::floor(std::min(std::min(imagePoints[0].x, imagePoints[1].x), std::min(imagePoints[2].x, imagePoints[3].x)));
    colMax = 1 + std::floor(std::max(std::max(imagePoints[0].x, imagePoints[1].x), std::max(imagePoints[2].x, imagePoints[3].x)));
    rowMin = std::floor(std::min(std::min(imagePoints[0].y, imagePoints[1].y), std::min(imagePoints[2].y, imagePoints[3].y)));
    rowMax = 1 + std::floor(std::max(std::max(imagePoints[0].y, imagePoints[1].y), std::max(imagePoints[2].y, imagePoints[3].y)));

    if ((colMin > imageWidth) || (colMax < 0) || (rowMin > imageHeight) || (rowMax < 0)) {
        throw std::runtime_error("The marker is not is the field of view");
    }

    double maxConfusionRadius = 0.0;
    for (int i = 0; i < 4; i++) {
        double distance = tvec(2) + rotationMatrix.at<double>(2, 0) * markerPoints[i].x + rotationMatrix.at<double>(2, 1) * markerPoints[i].y + rotationMatrix.at<double>(2, 2) * markerPoints[i].z;
        double confusionRadius = circleOfConfusionRadiusInPixels(distance);
        maxConfusionRadius = std::max(confusionRadius, maxConfusionRadius);
    }
    maxConfusionRadius = std::max(maxConfusionRadius + 1.0, MIN_PIXEL_MARGIN);

    colMin -= (int) (maxConfusionRadius);
    colMax += (int) (maxConfusionRadius);
    rowMin -= (int) (maxConfusionRadius);
    rowMax += (int) (maxConfusionRadius);

    colMin = std::max(0, colMin);
    colMax = std::min(imageWidth, colMax);
    rowMin = std::max(0, rowMin);
    rowMax = std::min(imageHeight, rowMax);

    if ((colMax - colMin < 3) || (rowMax - rowMin < 3)) {
        throw std::runtime_error("The marker is too small");
    }
}

void ThinLensCamera::computeSharpImageAndDepthMap() {
    minPatternDistance = DBL_MAX;
    maxPatternDistance = -DBL_MAX;

    distanceMap = cv::Mat::zeros(cv::Size(imageWidth, imageHeight), CV_64F);
    confusionMap = cv::Mat::zeros(cv::Size(imageWidth, imageHeight), CV_64F);
    intensityMap = cv::Mat::ones(cv::Size(imageWidth, imageHeight), CV_64F) * backgroundIntensity;
    countMap = cv::Mat::zeros(cv::Size(imageWidth, imageHeight), CV_32S);

#pragma omp parallel for num_threads(omp_get_num_procs())
    for (int row = rowMin; row < rowMax; row++) {
        for (int col = colMin; col < colMax; col++) {

            countMap.at<int>(row, col) = 1;

            double xSensor = (distortionMapX.at<float>(row, col) - principalPointX) * pixelPitch * inverseRectificationCoeff;
            double ySensor = (distortionMapY.at<float>(row, col) - principalPointY) * pixelPitch * inverseRectificationCoeff;
            Eigen::Vector3d raysCrossingPoint(xSensor * focusDistanceOverLensToSensorDistance, ySensor * focusDistanceOverLensToSensorDistance, focusDistance);

            Eigen::Vector3d intersectionPoint = -raysCrossingPoint * markerDistance / markerNormal.dot(raysCrossingPoint);
            double distance = intersectionPoint.z();
            distanceMap.at<double>(row, col) = distance;
            confusionMap.at<double>(row, col) = circleOfConfusionRadiusInPixels(distance);

            int colIntersection = std::floor((markerOriginX + mTc(0, 0) * intersectionPoint.x() + mTc(0, 1) * intersectionPoint.y() + mTc(0, 2) * intersectionPoint.z() + mTc(0, 3)) / markerPixelWidth);
            int rowIntersection = std::floor((markerOriginY + mTc(1, 0) * intersectionPoint.x() + mTc(1, 1) * intersectionPoint.y() + mTc(1, 2) * intersectionPoint.z() + mTc(1, 3)) / markerPixelHeight);

            if (colIntersection >= 0 && colIntersection < markerBitmap.cols && rowIntersection >= 0 && rowIntersection < markerBitmap.rows) {
                intensityMap.at<double>(row, col) = markerBitmap.at<double>(rowIntersection, colIntersection);
                if (distance > maxPatternDistance) {
                    maxPatternDistance = distance;
                }
                if (distance < minPatternDistance) {
                    minPatternDistance = distance;
                }
            } else {
                intensityMap.at<double>(row, col) = backgroundIntensity;
            }
        }
    }
}

void ThinLensCamera::computeEdgeMaps() {
    edgeMap = cv::Mat::ones(cv::Size(imageWidth, imageHeight), CV_8U) * 255;
    distanceToEdgeMap = cv::Mat::zeros(cv::Size(imageWidth, imageHeight), CV_32F);

#pragma omp parallel for num_threads(omp_get_num_procs())
    for (int row = rowMin + 1; row < rowMax - 1; row++) {
        for (int col = colMin + 1; col < colMax - 1; col++) {
            double deltaLeft = std::fabs(intensityMap.at<double>(row, col) - intensityMap.at<double>(row, col - 1));
            double deltaRight = std::fabs(intensityMap.at<double>(row, col + 1) - intensityMap.at<double>(row, col));
            double deltaUp = std::fabs(intensityMap.at<double>(row, col) - intensityMap.at<double>(row - 1, col));
            double deltaDown = std::fabs(intensityMap.at<double>(row + 1, col) - intensityMap.at<double>(row, col));
            double deltaMax = std::max(std::max(deltaLeft, deltaRight), std::max(deltaUp, deltaDown));
            edgeMap.at<char>(row, col) = (char) (255 - 255 * deltaMax);
        }
    }
    cv::distanceTransform(edgeMap(cv::Rect(colMin, rowMin, colMax - colMin, rowMax - rowMin)), distanceToEdgeMap(cv::Rect(colMin, rowMin, colMax - colMin, rowMax - rowMin)), cv::DIST_L2, 0);
}

void ThinLensCamera::refineImageWithAdaptiveSampling() {
    int sqrtNbRays = sqrt(maxIntensityValue);

#pragma omp parallel for num_threads(omp_get_num_procs())
    for (int row = rowMin; row < rowMax; row++) {
        for (int col = colMin; col < colMax; col++) {
            
            int pixelIndex = col + row * colMax;
            
            if (distanceToEdgeMap.at<float>(row, col) < confusionMap.at<double>(row, col) || distanceToEdgeMap.at<float>(row, col) < 1.0) {

                for (int colLens = 0; colLens < sqrtNbRays; colLens++) {
                    for (int rowLens = 0; rowLens < sqrtNbRays; rowLens++) {

                        countMap.at<int>(row, col) += 1;
                        
                        int sobolIndex = colLens + (rowLens * sqrtNbRays) + pixelIndex*sqrtNbRays*sqrtNbRays;
                        //int sobolIndex = colLens + (rowLens * sqrtNbRays); // same sampling for every pixels

                        double rx = sobol::sample(sobolIndex, 1) - 0.5;
                        double ry = sobol::sample(sobolIndex, 2) - 0.5;

                        double xSensor = (distortionMapX.at<float>(row, col) - principalPointX + rx) * pixelPitch * inverseRectificationCoeff;
                        double ySensor = (distortionMapY.at<float>(row, col) - principalPointY + ry) * pixelPitch * inverseRectificationCoeff;
                        Eigen::Vector3d raysCrossingPoint(xSensor * focusDistanceOverLensToSensorDistance, ySensor * focusDistanceOverLensToSensorDistance, focusDistance);

                        Eigen::Vector3d lensPoint(lensRadius * (colLens * 2.0 / (sqrtNbRays - 1) - 1.0), lensRadius * (rowLens * 2.0 / (sqrtNbRays - 1) - 1.0), 0.0);
                        concentricMapping(lensPoint.x(), lensPoint.y());

                        Eigen::Vector3d rayDirection = raysCrossingPoint - lensPoint;

                        double denom = markerNormal.dot(rayDirection);
                        double t = -(markerNormal.dot(lensPoint) + markerDistance) / denom;

                        Eigen::Vector3d intersectionPoint = lensPoint + rayDirection * t;
                        double depth = intersectionPoint.z();

                        int colIntersection = std::floor((markerOriginX + mTc(0, 0) * intersectionPoint.x() + mTc(0, 1) * intersectionPoint.y() + mTc(0, 2) * intersectionPoint.z() + mTc(0, 3)) / markerPixelWidth);
                        int rowIntersection = std::floor((markerOriginY + mTc(1, 0) * intersectionPoint.x() + mTc(1, 1) * intersectionPoint.y() + mTc(1, 2) * intersectionPoint.z() + mTc(1, 3)) / markerPixelHeight);

                        if (colIntersection >= 0 && colIntersection < markerBitmap.cols && rowIntersection >= 0 && rowIntersection < markerBitmap.rows) {
                            intensityMap.at<double>(row, col) += markerBitmap.at<double>(rowIntersection, colIntersection);
                        } else {
                            intensityMap.at<double>(row, col) += backgroundIntensity;
                        }
                    }
                }
                if (countMap.at<int>(row, col) > 1) {
                    intensityMap.at<double>(row, col) /= countMap.at<int>(row, col);
                }
            }
        }
    }
}

void ThinLensCamera::addDiffractionBlur() {
    cv::Mat kernel;
    discreteAiryKernel(airyDiskRadiusInPixels(), kernel);
    cv::filter2D(intensityMap, intensityMap, CV_64F, kernel);
}

double ThinLensCamera::circleOfConfusionRadiusInPixels(double objectDistance) {
        return std::fabs(lensRadius * focalLength * (objectDistance - focusDistance) / objectDistance / (focalLength + focusDistance) / pixelPitch);
    }

double ThinLensCamera::airyDiskRadiusInPixels() const {
    return airyDiskRadius() / pixelPitch;
}

double ThinLensCamera::airyDiskRadius() const {
    return 0.5 * AIRY_FIRST_ZERO_RADIUS * lightWaveLength * lensToSensorDistance / lensRadius;
}

void ThinLensCamera::applyGammaCorrection() {
    for (int row = rowMin; row < rowMax; row++) {
        for (int col = colMin; col < colMax; col++) {
            double lightIntensity = intensityMap.at<double>(row, col);
            intensityMap.at<double>(row, col) = gammaCorrection(lightIntensity);
        }
    }
}

void ThinLensCamera::quantifyOutputImage(cv::Mat & outputImage) {
    if (bitDepth <= 8) {
        intensityMap.convertTo(outputImage, CV_8U, maxIntensityValue);
    } else {
        intensityMap.convertTo(outputImage, CV_16U, maxIntensityValue);
    }
}

double ThinLensCamera::angleOfViewInDeg() const {
    double diagonal = pixelPitch * sqrt(imageHeight * imageHeight + imageWidth * imageWidth);
    return 360 * atan2(diagonal, 2 * focalLength) / PI;
}

double ThinLensCamera::depthOfField() const {
    return farDepthOfFieldLimit() - nearDepthOfFieldLimit();
}

double ThinLensCamera::hyperfocalDistance() const {
    return focalLength * focalLength / fNumber / pixelPitch;
}

double ThinLensCamera::nearDepthOfFieldLimit() const {
    double hyperfocalDistance = focalLength * focalLength / fNumber / pixelPitch;
    return hyperfocalDistance * focusDistance / (hyperfocalDistance + focusDistance);
}

double ThinLensCamera::farDepthOfFieldLimit() const {
    double hyperfocalDistance = focalLength * focalLength / fNumber / pixelPitch;
    if (hyperfocalDistance > focusDistance) {
        return hyperfocalDistance * focusDistance / (hyperfocalDistance - focusDistance);
    } else {
        return INFINITY;
    }
}

std::string ThinLensCamera::toString() const {
    std::ostringstream os;
    os << brand << std::endl;
    os << "  | resolution: " << imageWidth << "x" << imageHeight << " px" << std::endl;
    os << "  | pixel pitch: " << pixelPitch << " " << unit << std::endl;
    os << "  | color depth: " << bitDepth << " bits" << std::endl;
    os << "  | focal length: " << focalLength << " " << unit << std::endl;
    os << "  | angle of view: " << angleOfViewInDeg() << " deg" << std::endl;
    os << "  | f-number: " << fNumber << std::endl;
    os << "  | aperture: " << focalLength / fNumber << " " << unit << std::endl;
    os << "  | focus distance: " << focusDistance << " " << unit << std::endl;
    os << "  | hyperfocal distance: " << hyperfocalDistance() << " " << unit << std::endl;
    os << "  | near depth of field limit: " << nearDepthOfFieldLimit() << " " << unit << std::endl;
    os << "  | far depth of field limit: " << farDepthOfFieldLimit() << " " << unit << std::endl;
    os << "  | airy disk radius: " << airyDiskRadiusInPixels() << " px" << std::endl;

    return os.str();
}

std::ostream &operator<<(std::ostream &os, const ThinLensCamera & camera) {
    os << "Thin-lens camera model based on " << camera.toString();
    return os;
}

