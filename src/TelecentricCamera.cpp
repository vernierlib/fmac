/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "TelecentricCamera.hpp"

TelecentricCamera::TelecentricCamera(const std::string &ymlFilename, const std::string &bitmapFilename) {
    readCameraParameters(ymlFilename);
    readMarkerBitmap(bitmapFilename);
}

void TelecentricCamera::render(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &outputImage) {
    checkParameters();
    computeFrameTransforms(rvec, tvec);
    computeRayTracingMetricParameters();
    computeRegionOfInterest(rvec, tvec);
    computeSharpImageAndDepthMap();
    computeEdgeMaps();
    refineImageWithAdaptiveSampling();
    addDiffractionBlur();
    quantifyOutputImage(outputImage);
}

void TelecentricCamera::showMaps() {
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

void TelecentricCamera::readCameraParameters(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not find or read the camera parameter file: " + filename);
    }
    // cv::FileNode cameraBrandNode = file["camera_brand"];
    // if (cameraBrandNode.empty()) {
    //     std::cout << "'camera_brand' n'est pas présent dans le fichier YAML." << std::endl;
    // } else {
    //     // Récupérer la valeur en tant que double
    //     double focalLength = (double)focalLengthNode;
    //     std::cout << "Longueur focale : " << focalLength << std::endl;
    // }
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
    file["model"] >> model;
    file["objective_lens"] >> objectiveLens;
    file.release();
}

void TelecentricCamera::checkParameters() {
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

void TelecentricCamera::writeCameraParameters(const std::string &filename) {
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
    file << "model" << model;
    file << "objective_lens" << objectiveLens;
    file.release();
}

void TelecentricCamera::readMarkerBitmap(const std::string &bitmapFilename) {
    markerBitmap = cv::imread(bitmapFilename, cv::IMREAD_GRAYSCALE);
    if (markerBitmap.empty()) {
        throw std::runtime_error("Could not find or read the marker bitmap: " + bitmapFilename);
    }
    markerBitmap.convertTo(markerBitmap, CV_64F, 1);
    cv::normalize(markerBitmap, markerBitmap, 1.0, 0, cv::NORM_MINMAX);
}

std::ostream &operator<<(std::ostream &os, const TelecentricCamera &camera) {
    os << camera.model << " " << camera.imageWidth << "x" << camera.imageHeight << " " << camera.bitDepth << "bits f/" << camera.fNumber << " fd:" << camera.focusDistance << camera.unit;
    return os;
}

void TelecentricCamera::computeFrameTransforms(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
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
    // cTm.block<3,3>(0,0)=cTm.block<3,3>(0,0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    mTc = cTm.inverse();
}

void TelecentricCamera::computeRayTracingMetricParameters() {

    maxIntensityValue = 1 << bitDepth;

    principalPointX = cameraMatrix.at<double>(0, 2);
    principalPointY = cameraMatrix.at<double>(1, 2);

    focalLength = 0.5 * pixelPitch * (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1));
    lensRadius = 0.5 * focalLength / fNumber;
    lensToSensorDistance = focalLength * focusDistance / (focusDistance - focalLength);

    markerPixelWidth = markerWidth / markerBitmap.cols;
    markerPixelHeight = markerHeight / markerBitmap.rows;

    markerNormal = Eigen::Vector3d(mTc(2, 0), mTc(2, 1), mTc(2, 2));
    markerDistance = mTc(2, 3);
    focusDistanceOverLensToSensorDistance = focusDistance / lensToSensorDistance;

    Eigen::Vector3d rayDirection(0.0, 0.0, 1.0);
    double cosAngle = markerNormal.dot(rayDirection);
    if (cosAngle < cos(MAX_TILT_ANGLE_IN_DEG * M_PI / 180.0)) {
        throw std::runtime_error("The marker is not visible (tilt angle too large)");
    }

    inverseRectificationCoeff = focusDistance / (focusDistance - focalLength);
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

    colMin -= (int)(maxConfusionRadius);
    colMax += (int)(maxConfusionRadius);
    rowMin -= (int)(maxConfusionRadius);
    rowMax += (int)(maxConfusionRadius);

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

            double xSensor = (col - principalPointX ) * pixelPitch;
            double ySensor = (row - principalPointY ) * pixelPitch;
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

void TelecentricCamera::computeEdgeMaps() {
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
            edgeMap.at<char>(row, col) = (char)(255 - 255 * deltaMax);
        }
    }
    cv::distanceTransform(edgeMap(cv::Rect(colMin, rowMin, colMax - colMin, rowMax - rowMin)), distanceToEdgeMap(cv::Rect(colMin, rowMin, colMax - colMin, rowMax - rowMin)), cv::DIST_L2, 0);
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

                        double xSensor = (col - principalPointX + rx) * pixelPitch;
                        double ySensor = (row - principalPointY + ry) * pixelPitch;
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

void TelecentricCamera::addDiffractionBlur() {
    double sigma = airyDiskRadiusInPixels() / 3.0;
    int kernelSize = 2 * (int)(sigma + 1.0) + 1;
    cv::GaussianBlur(intensityMap, intensityMap, cv::Size(kernelSize, kernelSize), sigma);
}

void TelecentricCamera::quantifyOutputImage(cv::Mat &outputImage) {
    if (bitDepth <= 8) {
        intensityMap.convertTo(outputImage, CV_8U, maxIntensityValue);
    } else {
        intensityMap.convertTo(outputImage, CV_16U, maxIntensityValue);
    }
}
