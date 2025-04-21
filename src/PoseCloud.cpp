/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "PoseCloud.hpp"

PoseCloud::PoseCloud(int seed, int poseCount, double yawMin, double yawMax, double pitchMin, double pitchMax, double rollMin, double rollMax, double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) {
    this->seed = seed;
    this->poseCount = poseCount;
    this->yawMin = yawMin;
    this->yawMax = yawMax;
    this->pitchMin = pitchMin;
    this->pitchMax = pitchMax;
    this->rollMin = rollMin;
    this->rollMax = rollMax;
    this->xMin = xMin;
    this->xMax = xMax;
    this->yMin = yMin;
    this->yMax = yMax;
    this->zMin = zMin;
    this->zMax = zMax;
    rvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    tvec = cv::Mat::zeros(poseCount, 3, CV_64F);
    rpy = cv::Mat::zeros(poseCount, 3, CV_64F);
    checkParameters();
    generate();
}

PoseCloud::PoseCloud(const std::string &filename) {
    read(filename);
}

void PoseCloud::checkParameters() {
    assert(seed >= 0);
    assert(poseCount > 0);
    assert(yawMin <= yawMax);
    assert(pitchMin <= pitchMax);
    assert(rollMin <= rollMax);
    assert(xMin <= xMax);
    assert(yMin <= yMax);
    assert(zMin > 0 && zMin <= zMax);
    assert(rvec.rows == poseCount && rvec.cols == 3);
    assert(tvec.rows == poseCount && tvec.cols == 3);
    assert(rpy.rows == poseCount && rpy.cols == 3);
}

void PoseCloud::generate() {

    // Initialize Halton sampler with seed
    srand48(seed);
    Halton_sampler haltonSampler;
    haltonSampler.init_faure();

    // Compute extrinsic parameters
    for (int k = 0; k < poseCount; k++) {

        double yaw = scale(haltonSampler.sample(0, k), yawMin, yawMax);
        double pitch = scale(haltonSampler.sample(1, k), pitchMin, pitchMax);
        double roll = scale(haltonSampler.sample(2, k), rollMin, rollMax);
        double z = scale(haltonSampler.sample(3, k), zMin, zMax);
        double x = scaleWrtDistance(haltonSampler.sample(4, k), xMin, xMax, z, zMin);
        double y = scaleWrtDistance(haltonSampler.sample(5, k), yMin, yMax, z, zMin);

        rpy.at<double>(k, 0) = roll;
        rpy.at<double>(k, 1) = pitch;
        rpy.at<double>(k, 2) = yaw;

        cv::Mat rmat;
        taitBryanAnglesToRotationMatrix(yaw, pitch, roll, rmat);
        cv::Mat rv;
        Rodrigues(rmat, rv);

        rvec.at<double>(k, 0) = rv.at<double>(0);
        rvec.at<double>(k, 1) = rv.at<double>(1);
        rvec.at<double>(k, 2) = rv.at<double>(2);

        tvec.at<double>(k, 0) = x;
        tvec.at<double>(k, 1) = y;
        tvec.at<double>(k, 2) = z;
    }
}

std::ostream &operator<<(std::ostream &os, const PoseCloud &cloud) {
    os << "seed:" << cloud.seed << " count:" << cloud.poseCount << " yaw:[" << cloud.yawMin << ";" << cloud.yawMax << "] " << " pitch:[" << cloud.pitchMin << ";" << cloud.pitchMax << "] " << " roll:[" << cloud.rollMin << ";" << cloud.rollMax << "] " << " x:[" << cloud.xMin << ";" << cloud.xMax << "] " << " y:[" << cloud.yMin << ";" << cloud.yMax << "] " << " z:[" << cloud.zMin << ";" << cloud.zMax << "] ";
    return os;
}

void PoseCloud::printPoses() {
    for (int k = 0; k < rpy.rows; k++) {
        std::cout << "[" << k << "] yaw: " << rpy.at<double>(k, 2) << ", pitch: " << rpy.at<double>(k, 1) << ", roll: " << rpy.at<double>(k, 0) << ", x: " << tvec.at<double>(k, 0) << ", y: " << tvec.at<double>(k, 1) << ", z: " << tvec.at<double>(k, 2) << std::endl;
    }
}

void PoseCloud::write(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not write the file: " + filename);
    }
    file << "seed" << seed;
    file << "poseCount" << poseCount;
    file << "yawMin" << yawMin;
    file << "yawMax" << yawMax;
    file << "pitchMin" << pitchMin;
    file << "pitchMax" << pitchMax;
    file << "rollMin" << rollMin;
    file << "rollMax" << rollMax;
    file << "xMin" << xMin;
    file << "xMax" << xMax;
    file << "yMin" << yMin;
    file << "yMax" << yMax;
    file << "zMin" << zMin;
    file << "zMax" << zMax;
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
    file["yawMin"] >> yawMin;
    file["yawMax"] >> yawMax;
    file["pitchMin"] >> pitchMin;
    file["pitchMax"] >> pitchMax;
    file["rollMin"] >> rollMin;
    file["rollMax"] >> rollMax;
    file["xMin"] >> xMin;
    file["xMax"] >> xMax;
    file["yMin"] >> yMin;
    file["yMax"] >> yMax;
    file["zMin"] >> zMin;
    file["zMax"] >> zMax;
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

void PoseCloud::taitBryanAnglesToRotationMatrix(double yaw, double pitch, double roll, cv::Mat &result) {
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

double PoseCloud::scale(double value, double min, double max) {
    return min + (max - min) * value;
}

double PoseCloud::scaleWrtDistance(double x, double xMin, double xMax, double z, double zMin) {
    double min = z * xMin / zMin;
    double max = z * xMax / zMin;
    return scale(x, min, max);
}

// // Checks if a matrix is a valid rotation matrix.
// bool isRotationMatrix(Mat &R)
// {
//     Mat Rt;
//     transpose(R, Rt);
//     Mat shouldBeIdentity = Rt * R;
//     Mat I = Mat::eye(3,3, shouldBeIdentity.type());

//     return  norm(I, shouldBeIdentity) < 1e-6;

// }

// // Calculates rotation matrix to euler angles
// // The result is the same as MATLAB except the order
// // of the euler angles ( x and z are swapped ).
// Vec3f rotationMatrixToEulerAngles(Mat &R)
// {

//     assert(isRotationMatrix(R));

//     float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

//     bool singular = sy < 1e-6; // If

//     float x, y, z;
//     if (!singular)
//     {
//         x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
//         y = atan2(-R.at<double>(2,0), sy);
//         z = atan2(R.at<double>(1,0), R.at<double>(0,0));
//     }
//     else
//     {
//         x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
//         y = atan2(-R.at<double>(2,0), sy);
//         z = 0;
//     }
//     return Vec3f(x, y, z);

// }