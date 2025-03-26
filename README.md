# Fiducial Marker Accuracy Comparator

[![Contact](https://img.shields.io/badge/contact-form-green.svg)](https://projects.femto-st.fr/vernier/contact) 
[![GPL](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)
![Static Badge](https://img.shields.io/badge/C++-hand coded-orange)


The Fiducial Marker Accuracy Comparator is an open-source C++ library for rendering synthetic images of fiducial markers like ArUco, AprilTag, ARTag, STag, TopoTag and all others.

## Installation

### Windows instructions

First you have to install OpenCV, Eigen and OpenMP:

1. Download the OpenCV release from [https://opencv.org/releases/](https://github.com/opencv/opencv/releases/download/4.7.0/opencv-4.7.0-windows.exe)
2. Unpack the self-extracting archive in a local directory, for example `C:\lib\opencv`
3. Set the environment variables as follows:
	- Go to `Control Panel -> System and Security -> System Advanced System Settings -> Environment Variables`
	- Add a new user variable with name `OpenCV_DIR` and value `C:\lib\opencv\build`
<!---     - Edit the user variable Path and add a new directory `C:\lib\opencv\build\x64\vc16\bin` (location of the dll files) -->

Then, install [CMake](https://cmake.org/) and:

1. Open the directory with CMake
2. Generate the project for your preferred EDI
3. Restart the computer for system path updating
4. Open the generated project

### Linux instructions

With Linux, some dependencies must be installed first using apt-get:

	> sudo apt-get install cmake eigen opencv llvm libomp

Then, open a terminal and go to the directory of the package:

	> mkdir build
	> cd build
	> cmake ..
	> make

### OSX instructions

With OSX, some dependencies must be installed first using homebrew:

	% brew install cmake eigen opencv llvm libomp

Sometimes it is necessary to force the links creation

	% brew link --force libomp

Then, open a terminal and go to the directory of the package

	% mkdir build
	% cd build
	% cmake ..
	% make
	


## About

This software is part of the Vernier Library that is written and maintained by researchers with the FEMTO-ST Institute located in Besançon, France.

Authors: Guillaume J. Laurent, Patrick Sandoz

Contact: [vernier@femto-st.fr](mailto:vernier@femto-st.fr)

Copyright (c) 2025 ENSMM, UMLP, CNRS.

The software is built over third party libraries:

  - [OpenCV](http://opencv.org/)   
  - [Eigen](http://eigen.tuxfamily.org)
  - [OpenMP](https://www.openmp.org/)
  - the samplers are from [Leonhard Gruenschloss](http://gruenschloss.org/) 


## Licence

The Vernier Library is Free Software in the technical sense defined by the Free Software Foundation, and is distributed under the terms of the [GNU General Public License](LICENSE.txt). 

Non-free licenses are also available for companies that wish to use the Vernier library in their products but are unwilling to release their software under the GPL (which would require them to release source code and allow free redistribution). Contact us for more details: [vernier@femto-st.fr](mailto:vernier@femto-st.fr)


