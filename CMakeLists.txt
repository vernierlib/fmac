# cmake needs this line
cmake_minimum_required(VERSION 3.10)

# Project name
project(fmac)

# Find Eigen
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 library status:")
message(STATUS "    version: ${Eigen3_VERSION}")
message(STATUS "    config: ${Eigen3_DIR}")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")
add_definitions(-I${EIGEN3_INCLUDE_DIR})

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    libraries: ${OpenCV_LIBS}")

# Find OpenMP (optional)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
message(STATUS "OpenMP_CXX library status:")
message(STATUS "    version: ${OpenMP_CXX_VERSION}")
message(STATUS "    include path: ${OpenMP_CXX_INCLUDE_DIRS}")
message(STATUS "    library path: ${OpenMP_CXX_LIBRARIES}")
message(STATUS "    libraries: ${OpenMP_CXX_LIB_NAMES}")
endif()

# Executable target built
file(GLOB FILES *.cpp)
foreach(file ${FILES})
  get_filename_component(filename ${file} NAME) 
  get_filename_component(targetname ${file} NAME_WLE) 
  add_executable(${targetname} ${filename} src/ProjectiveCamera.cpp src/PoseCloud.cpp 3rdparty/gruenschloss/kuo-2d-proj/sobol.cpp)
  target_include_directories(${targetname} PUBLIC "${CMAKE_SOURCE_DIR}/include")
  target_include_directories(${targetname} PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/kuo-2d-proj")
  target_include_directories(${targetname} PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/halton")
  target_link_libraries(${targetname} PUBLIC ${OpenCV_LIBS})
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${targetname} PUBLIC OpenMP::OpenMP_CXX)
  endif()
endforeach(file)
#target_include_directories(poseGeneration PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/halton")

# Copy data in the binary
add_subdirectory(data/opencv)
add_subdirectory(data/matlab)
add_subdirectory(data/aruco)
