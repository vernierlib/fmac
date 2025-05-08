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
    checkParameters();

    // Initialize Halton sampler with seed
    srand48(seed);
    Halton_sampler haltonSampler;
    haltonSampler.init_faure();

    // Compute poses
    for (int k = 0; k < poseCount; k++) {
        
        double yaw = scale(haltonSampler.sample(0, k), box.yawMin, box.yawMax);
        double pitch = scale(haltonSampler.sample(1, k), box.pitchMin, box.pitchMax);
        double roll = scale(haltonSampler.sample(2, k), box.rollMin, box.rollMax);
        double z = scale(haltonSampler.sample(3, k), box.zMin, box.zMax);
        double x = scale(haltonSampler.sample(4, k), box.xMin, box.xMax);
        double y = scale(haltonSampler.sample(5, k), box.yMin, box.yMax);

        setTaitBryanAngles(k, roll, pitch, yaw);
        //        rpy.at<double>(k, 0) = roll;
        //        rpy.at<double>(k, 1) = pitch;
        //        rpy.at<double>(k, 2) = yaw;
        //
        //        cv::Mat rmat;
        //        taitBryanAnglesToRotationMatrix(roll, pitch, yaw, rmat);
        //        cv::Mat rv;
        //        Rodrigues(rmat, rv);
        //
        //        rvec.at<double>(k, 0) = rv.at<double>(0);
        //        rvec.at<double>(k, 1) = rv.at<double>(1);
        //        rvec.at<double>(k, 2) = rv.at<double>(2);

        setTVec(k, x, y, z);
        //        tvec.at<double>(k, 0) = x;
        //        tvec.at<double>(k, 1) = y;
        //        tvec.at<double>(k, 2) = z;
    }
}

PoseCloud::PoseCloud(int seed, int poseCount, PoseBox box, const ThinLensCamera & camera) {
    this->seed = seed;
    this->poseCount = poseCount;
    this->box = box;

    // Initilize the matrices
    rvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    tvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    rpy = cv::Mat::zeros(poseCount, 3, CV_64F);
    checkParameters();

    // Initialize Halton sampler with seed
    srand48(seed);
    Halton_sampler haltonSampler;
    haltonSampler.init_faure();

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
//                rpy.at<double>(k, 0) = roll;
//                rpy.at<double>(k, 1) = pitch;
//                rpy.at<double>(k, 2) = yaw;
//        
//                cv::Mat rmat;
//                taitBryanAnglesToRotationMatrix(roll, pitch, yaw, rmat);
//                cv::Mat rv;
//                cv::Rodrigues(rmat, rv);
//        
//                rvec.at<double>(k, 0) = rv.at<double>(0);
//                rvec.at<double>(k, 1) = rv.at<double>(1);
//                rvec.at<double>(k, 2) = rv.at<double>(2);

        setTVec(k, x, y, z);
//                tvec.at<double>(k, 0) = x;
//                tvec.at<double>(k, 1) = y;
//                tvec.at<double>(k, 2) = z;
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
}

void PoseCloud::write(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not write the file: " + filename);
    }
    file << "seed" << seed;
    file << "poseCount" << poseCount;
    file << "rvec" << rvec;
    file << "tvec" << tvec;
    file << "rpy" << rpy;
    file.release();
}

void PoseCloud::read(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not find or read the file: " + filename);
    }
    file["seed"] >> seed;
    file["poseCount"] >> poseCount;
    file["rvec"] >> rvec;
    file["tvec"] >> tvec;
    file["rpy"] >> rpy;
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

void PoseCloud::draw(cv::Mat & image, const ThinLensCamera & camera) {
    for (int k = 0; k < getPoseCount(); k++) {
        cv::drawFrameAxes(image, camera.cameraMatrix, camera.distortionCoefficients, getRVec(k), getTVec(k), std::max(camera.markerWidth, camera.markerHeight), 1);
    }
}

std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud) {
//    for (int k = 0; k < cloud.rpy.rows; k++) {
//        os << "[" << k << "] yaw: " << cloud.rpy.at<double>(k, 2) << ", pitch: " << cloud.rpy.at<double>(k, 1) << ", roll: " << cloud.rpy.at<double>(k, 0) << ", x: " << cloud.tvec.at<double>(k, 0) << ", y: " << cloud.tvec.at<double>(k, 1) << ", z: " << cloud.tvec.at<double>(k, 2) << std::endl;
//    }
    os << "Cloud of " << cloud.poseCount << " poses in the box: " << cloud.box; 
    return os;
}

void PoseCloud::taitBryanAnglesToRotationMatrix(double roll, double pitch, double yaw, cv::Mat &result) {
    // Rotation matrix about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

    // Rotation matrix about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));

    // Rotation matrix about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1);

    // Combined rotation matrix
    result = R_z * R_y * R_x;
}

void PoseCloud::rotationMatrixToTaitBryanAngles(const cv::Mat & rmat, double & roll, double & pitch, double & yaw) {

    double sy = sqrt(rmat.at<double>(0, 0) * rmat.at<double>(0, 0) + rmat.at<double>(1, 0) * rmat.at<double>(1, 0));

    bool singular = sy < 1e-6;

    if (!singular) {
        roll = atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
        pitch = atan2(-rmat.at<double>(2, 0), sy);
        yaw = atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));
    } else {
        roll = atan2(-rmat.at<double>(1, 2), rmat.at<double>(1, 1));
        pitch = atan2(-rmat.at<double>(2, 0), sy);
        yaw = 0.0;
    }
}

double PoseCloud::scale(double value, double min, double max) {
    return min + (max - min) * value;
}

// // Checks if a matrix is a valid rotation matrix.
// // https://learnopencv.com/rotation-matrix-to-euler-angles/
// bool isRotationMatrix(Mat &R)
// {
//     Mat Rt;
//     transpose(R, Rt);
//     Mat shouldBeIdentity = Rt * R;
//     Mat I = Mat::eye(3,3, shouldBeIdentity.type());
//
//     return  norm(I, shouldBeIdentity) < 1e-6;
//
// }
