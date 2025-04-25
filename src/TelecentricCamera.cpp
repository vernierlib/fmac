/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "TelecentricCamera.hpp"

TelecentricCamera::TelecentricCamera(const std::string &ymlFilename, const std::string &bitmapFilename)
: ThinLensCamera(ymlFilename, bitmapFilename) {
}

void TelecentricCamera::computeRegionOfInterest(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    std::vector<cv::Point3d> markerCorners(4);
    markerCorners[0] = cv::Point3d(0.0 - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerCorners[1] = cv::Point3d(markerWidth - markerOriginX, 0.0 - markerOriginY, 0.0);
    markerCorners[2] = cv::Point3d(markerWidth - markerOriginX, markerHeight - markerOriginY, 0.0);
    markerCorners[3] = cv::Point3d(0.0 - markerOriginX, markerHeight - markerOriginY, 0.0);

    std::vector<cv::Point2d> imagePoints;
    imagePoints.resize(markerCorners.size());
    for (int k = 0; k < markerCorners.size(); k++) { // project corners
        Eigen::Vector4d corner, point;
        corner(0) = markerCorners[k].x;
        corner(1) = markerCorners[k].y;
        corner(2) = markerCorners[k].z;
        corner(3) = 1.0;
        point = cTm * corner;
        imagePoints[k].x = point(0) * focalLength / focusDistance / pixelPitch + principalPointX;
        imagePoints[k].y = point(1) * focalLength / focusDistance / pixelPitch + principalPointY;
    }
    // std::vector<cv::Point2d> imagePoints;
    // cv::projectPoints(markerCorners, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    colMin = std::floor(std::min(std::min(imagePoints[0].x, imagePoints[1].x), std::min(imagePoints[2].x, imagePoints[3].x)));
    colMax = 1 + std::floor(std::max(std::max(imagePoints[0].x, imagePoints[1].x), std::max(imagePoints[2].x, imagePoints[3].x)));
    rowMin = std::floor(std::min(std::min(imagePoints[0].y, imagePoints[1].y), std::min(imagePoints[2].y, imagePoints[3].y)));
    rowMax = 1 + std::floor(std::max(std::max(imagePoints[0].y, imagePoints[1].y), std::max(imagePoints[2].y, imagePoints[3].y)));

    if ((colMin > imageWidth) || (colMax < 0) || (rowMin > imageHeight) || (rowMax < 0)) {
        throw std::runtime_error("The marker is not is the field of view");
    }

    double maxConfusionRadius = 0.0;
    for (int i = 0; i < 4; i++) {
        double distance = tvec(2) + rotationMatrix.at<double>(2, 0) * markerCorners[i].x + rotationMatrix.at<double>(2, 1) * markerCorners[i].y + rotationMatrix.at<double>(2, 2) * markerCorners[i].z;
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

void TelecentricCamera::computeSharpImageAndDepthMap() {
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

            // double xSensor = (col - principalPointX ) * pixelPitch;
            // double ySensor = (row - principalPointY ) * pixelPitch;
            double xSensor = (distortionMapX.at<float>(row, col) - principalPointX) * pixelPitch * inverseRectificationCoeff;
            double ySensor = (distortionMapY.at<float>(row, col) - principalPointY) * pixelPitch * inverseRectificationCoeff;
            Eigen::Vector3d raysCrossingPoint(xSensor * focusDistanceOverLensToSensorDistance, ySensor * focusDistanceOverLensToSensorDistance, focusDistance);

            Eigen::Vector3d lensPoint(raysCrossingPoint.x(), raysCrossingPoint.y(), 0.0);
            Eigen::Vector3d rayDirection = raysCrossingPoint - lensPoint;

            double denom = markerNormal.dot(rayDirection);
            double t = -(markerNormal.dot(lensPoint) + markerDistance) / denom;

            Eigen::Vector3d intersectionPoint = lensPoint + rayDirection * t;
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

void TelecentricCamera::refineImageWithAdaptiveSampling() {
    int sqrtNbRays = sqrt(maxIntensityValue);

#pragma omp parallel for num_threads(omp_get_num_procs())
    for (int row = rowMin; row < rowMax; row++) {
        for (int col = colMin; col < colMax; col++) {

            if (distanceToEdgeMap.at<float>(row, col) < confusionMap.at<double>(row, col) || distanceToEdgeMap.at<float>(row, col) < 1.0) {

                for (int colLens = 0; colLens < sqrtNbRays; colLens++) {
                    for (int rowLens = 0; rowLens < sqrtNbRays; rowLens++) {

                        countMap.at<int>(row, col) += 1;

                        double rx = sobol::sample(colLens + (rowLens * sqrtNbRays), 1) - 0.5;
                        double ry = sobol::sample(colLens + (rowLens * sqrtNbRays), 2) - 0.5;

                        //double xSensor = (col - principalPointX + rx) * pixelPitch;
                        //double ySensor = (row - principalPointY + ry) * pixelPitch;
                        double xSensor = (distortionMapX.at<float>(row, col) - principalPointX + rx) * pixelPitch * inverseRectificationCoeff;
                        double ySensor = (distortionMapY.at<float>(row, col) - principalPointY + ry) * pixelPitch * inverseRectificationCoeff;
                        Eigen::Vector3d raysCrossingPoint(xSensor * focusDistanceOverLensToSensorDistance, ySensor * focusDistanceOverLensToSensorDistance, focusDistance);

                        Eigen::Vector3d stopPoint(lensRadius * (colLens * 2.0 / (sqrtNbRays - 1) - 1.0), lensRadius * (rowLens * 2.0 / (sqrtNbRays - 1) - 1.0), 0.0);
                        concentricMapping(stopPoint.x(), stopPoint.y());

                        Eigen::Vector3d lensPoint(raysCrossingPoint.x(), raysCrossingPoint.y(), 0.0);
                        //lensPoint += stopPoint * (lensToSensorDistance / (lensToSensorDistance - focalLength));
                        lensPoint += stopPoint * (focusDistance / focalLength);

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

std::ostream &operator<<(std::ostream &os, const TelecentricCamera &camera) {
    os << "Telecentric camera model based on " << camera.toString();
    return os;
}
