/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#include "PoseBox.hpp"

PoseBox::PoseBox() {
    this->yawMin = 0.0;
    this->yawMax = 0.0;
    this->pitchMin = 0.0;
    this->pitchMax = 0.0;
    this->rollMin = 0.0;
    this->rollMax = 0.0;
    this->xMin = 0.0;
    this->xMax = 0.0;
    this->yMin = 0.0;
    this->yMax = 0.0;
    this->zMin = 0.0;
    this->zMax = 0.0;
}

PoseBox::PoseBox(double yawMin, double yawMax, double pitchMin, double pitchMax, double rollMin, double rollMax, double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) {
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
}

PoseBox::PoseBox(const std::string &filename) {
    read(filename);
}

void PoseBox::checkParameters() {
    assert(yawMin <= yawMax);
    assert(pitchMin <= pitchMax);
    assert(rollMin <= rollMax);
    assert(xMin <= xMax);
    assert(yMin <= yMax);
    assert(zMin > 0 && zMin <= zMax);
}

void PoseBox::write(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not write the file: " + filename);
    }
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
    file.release();
}

void PoseBox::read(const std::string &filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) {
        throw std::runtime_error("Could not find or read the file: " + filename);
    }
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
    file.release();
    checkParameters();
}

std::ostream &operator<<(std::ostream &os, const PoseBox &box) {
    os << "yaw:[" << box.yawMin << ";" << box.yawMax << "] " << " pitch:[" << box.pitchMin << ";" << box.pitchMax << "] " << " roll:[" << box.rollMin << ";" << box.rollMax << "] " << " x:[" << box.xMin << ";" << box.xMax << "] " << " y:[" << box.yMin << ";" << box.yMax << "] " << " z:[" << box.zMin << ";" << box.zMax << "]";
    return os;
}