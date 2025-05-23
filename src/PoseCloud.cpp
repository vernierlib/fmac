/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "PoseCloud.hpp"

PoseCloud::PoseCloud(int seed, int poseCount, PoseBox box) {
    this->seed = seed;
    this->poseCount = poseCount;
    this->box = box;

    // Initilize the matrices
    rvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    tvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    rpy = cv::Mat::zeros(poseCount, 3, CV_64F);
    markerCorners = cv::Mat::zeros(poseCount, 8, CV_64F);
    checkParameters();

    // Initialize Halton sampler with seed
    Halton_sampler haltonSampler;
    std::mt19937 rng(seed);
    haltonSampler.init_random(rng);

    // Compute poses
    for (int k = 0; k < poseCount; k++) {

        double yaw = scale(haltonSampler.sample(0, k), box.yawMin, box.yawMax);
        double pitch = scale(haltonSampler.sample(1, k), box.pitchMin, box.pitchMax);
        double roll = scale(haltonSampler.sample(2, k), box.rollMin, box.rollMax);
        double z = scale(haltonSampler.sample(3, k), box.zMin, box.zMax);
        double x = scale(haltonSampler.sample(4, k), box.xMin, box.xMax);
        double y = scale(haltonSampler.sample(5, k), box.yMin, box.yMax);

        setTaitBryanAngles(k, roll, pitch, yaw);
        setTVec(k, x, y, z);
    }
}

PoseCloud::PoseCloud(int seed, int poseCount, PoseBox box, const ThinLensCamera & camera) {
    this->seed = seed;
    this->poseCount = poseCount;
    this->box = box;

    // Initialize the matrices
    rvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    tvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    rpy = cv::Mat::zeros(poseCount, 3, CV_64F);
    markerCorners = cv::Mat::zeros(poseCount, 8, CV_64F);
    checkParameters();

    // Initialize Halton sampler with seed
    Halton_sampler haltonSampler;
    std::mt19937 rng(seed);
    haltonSampler.init_random(rng);

    // Compute poses
    for (int k = 0; k < poseCount; k++) {

        double yaw = scale(haltonSampler.sample(0, k), box.yawMin, box.yawMax);
        double pitch = scale(haltonSampler.sample(1, k), box.pitchMin, box.pitchMax);
        double roll = scale(haltonSampler.sample(2, k), box.rollMin, box.rollMax);
        double z = scale(haltonSampler.sample(3, k), box.zMin, box.zMax);
        box.xMax = 0.5 * camera.imageWidth * camera.pixelPitch * z / camera.focalLength - std::hypot(camera.markerHeight, camera.markerWidth);
        box.xMin = -box.xMax;
        box.yMax = 0.5 * camera.imageHeight * camera.pixelPitch * z / camera.focalLength - std::hypot(camera.markerHeight, camera.markerWidth);
        box.yMin = -box.yMax;
        double x = scale(haltonSampler.sample(4, k), box.xMin, box.xMax);
        double y = scale(haltonSampler.sample(5, k), box.yMin, box.yMax);

        setTaitBryanAngles(k, roll, pitch, yaw);
        setTVec(k, x, y, z);
        setMarkerCorners(k, camera.markerCorners(getRVec(k), getTVec(k)));
    }
}

PoseCloud::PoseCloud(const std::string &filename) {
    read(filename);
}

void PoseCloud::checkParameters() {
    assert(seed >= 0);
    assert(poseCount > 0);
    assert(rvec.rows == poseCount && rvec.cols == 3);
    assert(tvec.rows == poseCount && tvec.cols == 3);
    assert(rpy.rows == poseCount && rpy.cols == 3);
    assert(markerCorners.rows == poseCount && markerCorners.cols == 8);
}

void PoseCloud::write(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not write the file: " + filename);
    }
    file << "seed" << seed;
    file << "pose_count" << poseCount;
    file << "rvec" << rvec;
    file << "tvec" << tvec;
    file << "rpy" << rpy;
    file << "marker_corners" << markerCorners;
    file.release();
}

void PoseCloud::read(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not find or read the file: " + filename);
    }
    file["seed"] >> seed;
    file["pose_count"] >> poseCount;
    file["rvec"] >> rvec;
    file["tvec"] >> tvec;
    file["rpy"] >> rpy;
    file["marker_corners"] >> markerCorners;
    file.release();
    checkParameters();
}

cv::Vec3d PoseCloud::getRVec(int index) {
    cv::Vec3d result;
    result(0) = rvec.at<double>(index, 0);
    result(1) = rvec.at<double>(index, 1);
    result(2) = rvec.at<double>(index, 2);
    return result;
}

cv::Vec3d PoseCloud::getTVec(int index) {
    cv::Vec3d result;
    result(0) = tvec.at<double>(index, 0);
    result(1) = tvec.at<double>(index, 1);
    result(2) = tvec.at<double>(index, 2);
    return result;
}

void PoseCloud::setTVec(int index, const cv::Mat & tvec) {
    this->tvec.at<double>(index, 0) = tvec.at<double>(0);
    this->tvec.at<double>(index, 1) = tvec.at<double>(1);
    this->tvec.at<double>(index, 2) = tvec.at<double>(2);
}

void PoseCloud::setTVec(int index, double x, double y, double z) {
    this->tvec.at<double>(index, 0) = x;
    this->tvec.at<double>(index, 1) = y;
    this->tvec.at<double>(index, 2) = z;
}

void PoseCloud::setTaitBryanAngles(int index, double roll, double pitch, double yaw) {
    rpy.at<double>(index, 0) = roll;
    rpy.at<double>(index, 1) = pitch;
    rpy.at<double>(index, 2) = yaw;

    cv::Mat rmat;
    taitBryanAnglesToRotationMatrix(roll, pitch, yaw, rmat);
    cv::Mat rv;
    cv::Rodrigues(rmat, rv);

    rvec.at<double>(index, 0) = rv.at<double>(0);
    rvec.at<double>(index, 1) = rv.at<double>(1);
    rvec.at<double>(index, 2) = rv.at<double>(2);
}

void PoseCloud::setRVec(int index, const cv::Mat & rvec) {
    this->rvec.at<double>(index, 0) = rvec.at<double>(0);
    this->rvec.at<double>(index, 1) = rvec.at<double>(1);
    this->rvec.at<double>(index, 2) = rvec.at<double>(2);

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    double roll, pitch, yaw;
    rotationMatrixToTaitBryanAngles(rmat, roll, pitch, yaw);

    rpy.at<double>(index, 0) = roll;
    rpy.at<double>(index, 1) = pitch;
    rpy.at<double>(index, 2) = yaw;
}

void PoseCloud::setMarkerCorners(int index, const std::vector<cv::Point2d> & corners) {
    assert(corners.size() == 4);
    markerCorners.at<double>(index, 0) = corners[0].x;
    markerCorners.at<double>(index, 1) = corners[0].y;
    markerCorners.at<double>(index, 2) = corners[1].x;
    markerCorners.at<double>(index, 3) = corners[1].y;
    markerCorners.at<double>(index, 4) = corners[2].x;
    markerCorners.at<double>(index, 5) = corners[2].y;
    markerCorners.at<double>(index, 6) = corners[3].x;
    markerCorners.at<double>(index, 7) = corners[3].y;
}

void PoseCloud::setMarkerCorners(int index, const std::vector<cv::Point2f> & corners) {
    assert(corners.size() == 4);
    markerCorners.at<double>(index, 0) = corners[0].x;
    markerCorners.at<double>(index, 1) = corners[0].y;
    markerCorners.at<double>(index, 2) = corners[1].x;
    markerCorners.at<double>(index, 3) = corners[1].y;
    markerCorners.at<double>(index, 4) = corners[2].x;
    markerCorners.at<double>(index, 5) = corners[2].y;
    markerCorners.at<double>(index, 6) = corners[3].x;
    markerCorners.at<double>(index, 7) = corners[3].y;
}

void PoseCloud::draw(cv::Mat & image, const ThinLensCamera & camera) {
    for (int k = 0; k < getPoseCount(); k++) {
        cv::drawFrameAxes(image, camera.cameraMatrix, camera.distortionCoefficients, getRVec(k), getTVec(k), std::max(camera.markerWidth, camera.markerHeight), 1);
    }
}

std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud) {
    os << "Cloud of " << cloud.poseCount << " poses in the box: " << cloud.box << std::endl;
    if (cloud.poseCount <= 100) {
        for (int k = 0; k < cloud.rpy.rows; k++) {
            os << "[" << k << "] yaw: " << cloud.rpy.at<double>(k, 2) << ", pitch: " << cloud.rpy.at<double>(k, 1) << ", roll: " << cloud.rpy.at<double>(k, 0) << ", x: " << cloud.tvec.at<double>(k, 0) << ", y: " << cloud.tvec.at<double>(k, 1) << ", z: " << cloud.tvec.at<double>(k, 2) << std::endl;
        }
    }
    return os;
}
