/*
 * This file is part of the VERNIER Library.
 *
 * Copyright (c) 2025 CNRS, ENSMM, UMLP.
 */

#ifndef POSE_BOX_HPP
#define POSE_BOX_HPP

#include <iostream>
#include <opencv2/core.hpp>

class PoseBox {
public:
    
    double yawMin;
    double yawMax;
    double pitchMin;
    double pitchMax;
    double rollMin;
    double rollMax;
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double zMin;
    double zMax;
    
    PoseBox();

    /// @brief Creates a box of poses from 12 boundaries.
    PoseBox(double yawMin, double yawMax, double pitchMin, double pitchMax, double rollMin, double rollMax, double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);

    /// @brief Creates a box of poses from a file.
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.xml.gz
    PoseBox(const std::string & filename);
    
    /// @brief Saves a box of poses in a file. 
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.gz
    void write(const std::string & filename);

    /// @brief Loads a box of poses from a file.
    /// @param filename Name of the file to open. Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON respectively). Also you can append .gz to work with compressed files, for example myHugeCloud.xml.gz
    void read(const std::string & filename);
    
    void checkParameters();

};

std::ostream &operator<<(std::ostream &os, const PoseBox &box);

#endif