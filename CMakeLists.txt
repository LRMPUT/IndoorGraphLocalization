cmake_minimum_required(VERSION 2.8)
project(GraphLocalization)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)

# Boost
FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# OpenCV Config
find_package(OpenCV REQUIRED)

if (CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
    message(STATUS "Found CHOLMOD and its dependencies")
    set(CHOLMOD_FOUND TRUE)
else ()
    set(CHOLMOD_FOUND FALSE)
endif ()

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/build
        ${EIGEN3_INCLUDE_DIR}
        ${CHOLMOD_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

ADD_DEFINITIONS(-DDEBUG -ggdb -O3 -Wall -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -ffast-math -fbranch-probabilities)

# Build 
add_executable(graph_localization
        src/local_g2o/vertex_one.cpp
        src/local_g2o/edge_one_prior.cpp
        src/local_g2o/edge_pdr.cpp
        src/local_g2o/edge_stepometer.cpp
        src/local_g2o/edge_wknn.cpp
        src/local_g2o/edge_wall.cpp
        src/local_g2o/register_types.cpp
        src/wifiLocalization.cpp
        src/stepometer.cpp
        src/DataReadWrite/dataReadWrite.cpp
        src/graphManager.cpp
        src/fastable/FastABLE.cpp
        src/ldb/ldb.cpp
        src/source.cpp)

target_link_libraries(graph_localization
        ${EIGEN3_LIBS}
        ${CHOLMOD_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_calibration_odom_laser.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_pcg.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_cli.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_slam2d_linear.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_structure_only.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_ext_freeglut_minimal.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_tutorial_slam2d.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_hierarchical.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_data.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_incremental.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_icp.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_interactive.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sba.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_interface.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sclam2d.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_opengl_helper.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sim3.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_parser.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam2d_addons.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_simulator.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam2d.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_cholmod.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam3d_addons.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam3d.so
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_dense.so)