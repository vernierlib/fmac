# Fiducial Marker Accuracy Comparator

[![Contact](https://img.shields.io/badge/contact-form-green.svg)](https://projects.femto-st.fr/vernier/contact) 
[![GPL](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

The Fiducial Marker Accuracy Comparator is an open-source C++ library for rendering synthetic images of fiducial markers like ArUco, AprilTag, ARTag, STag, TopoTag and all others.

## Installation

The library depends on [OpenCV](https://opencv.org/) 4, [Eigen](https://eigen.tuxfamily.org) 3 and, optionally, [OpenMP](https://www.openmp.org/) for multithreaded rendering. It is built with [CMake](https://cmake.org/) 3.10 or newer.

### Linux instructions

Install the dependencies with your package manager. On Debian or Ubuntu:

	$ sudo apt install build-essential cmake libeigen3-dev libopencv-dev

OpenMP comes with GCC, so nothing else is needed. If you build with Clang, also install `libomp-dev`.

Then, open a terminal in the directory of the package and run:

	$ cmake -S . -B build
	$ cmake --build build

### macOS instructions

Install the dependencies with [Homebrew](https://brew.sh/):

	% brew install cmake eigen opencv libomp

Then, open a terminal in the directory of the package and run:

	% cmake -S . -B build
	% cmake --build build

CMake looks for `libomp` in its Homebrew prefix. To use another OpenMP installation, pass `-DOpenMP_ROOT=<path>` when configuring.

### Windows instructions

Install [CMake](https://cmake.org/download/) and Visual Studio with the "Desktop development with C++" workload. OpenMP is included with the Visual Studio compiler.

Then, install OpenCV:

1. Download the Windows release of OpenCV 4.7 or newer (older releases lack the ArUco module needed by the `arucoDetection` example) from [https://opencv.org/releases/](https://opencv.org/releases/)
2. Unpack the self-extracting archive in a local directory, for example `C:\lib\opencv`

And Eigen:

1. Download the Eigen 3.4 source archive from [https://eigen.tuxfamily.org](https://eigen.tuxfamily.org) and unpack it, for example in `C:\src\eigen-3.4.0`
2. Install it from a terminal:

		> cmake -S C:\src\eigen-3.4.0 -B C:\src\eigen-build
		> cmake --install C:\src\eigen-build --prefix C:\lib\eigen

Set the environment variables so that CMake and the executables can find the libraries:

1. Go to `Control Panel -> System and Security -> System -> Advanced System Settings -> Environment Variables`
2. Add a new user variable with name `OpenCV_DIR` and value `C:\lib\opencv\build`
3. Add a new user variable with name `Eigen3_DIR` and value `C:\lib\eigen\share\eigen3\cmake`
4. Edit the user variable `Path` and add `C:\lib\opencv\build\x64\vc16\bin` (the folder containing the OpenCV DLLs, its name depends on the OpenCV version)
5. Log out and back in, or restart, so that the new variables are taken into account

Finally, open the directory of the package with CMake, generate the project for your IDE and open it.

## Running the examples

The build produces the `fmac` static library and a few example programs. CMake copies the `data` folder next to them, and the examples load their files from there using relative paths, so run them from the build directory:

	$ cd build
	$ ./simpleExample

| Example | Description |
|---|---|
| `simpleExample` | Renders an ArUco marker with a thin lens camera |
| `telecentricExample` | Renders a checkerboard with a telecentric camera |
| `projectionValidation` | Renders the OpenCV calibration views and compares them to the real images |
| `depthBlurValidation` | Renders a defocused checkerboard and compares it to a real image |
| `cloudGeneration` | Generates a cloud of random marker poses and renders an image for each one |
| `arucoDetection` | Detects the markers in the rendered images and saves the estimated poses |

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


