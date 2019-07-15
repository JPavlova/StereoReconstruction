cmake_minimum_required(VERSION 2.8)
project(stereo_reconstruction)

set(CMAKE_CXX_STANDARD)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")
add_definitions(-std=c++17)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# FreeImage
# find_package(FreeImage REQUIRED)
# include_directories(${FREEIMAGE_INCLUDE_DIRS})

#OpenMP
find_package(OpenMP)
include_directories(${OPENMP_INCLUDE_DIRS})

# Set files to be compiled
set(HEADER_FILES
    stereoimage.h
    prerequisites.h
    FreeImageHelper.h
    camerasensor.h
    exporter.h
    patchmatch.h
    patchmatchvectorfield.h
        )
set(SOURCE_FILES
    FreeImageHelper.cpp
    stereoimage.cpp
    patchmatch.cpp
    patchmatchvectorfield.cpp
        )

add_executable(stereo_reconstruction main.cpp ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(stereo_reconstruction PUBLIC OpenMP::OpenMP_CXX freeimage ${CERES_LIBRARIES} ${EIGEN_LIBRARIES} libstdc++fs.a)
