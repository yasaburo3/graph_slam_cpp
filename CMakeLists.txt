cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_cpp)
set( CMAKE_CXX_FLAGS "-std=c++14" )

list(APPEND CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/cmake_modules)
set(G2O_ROOT ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

# find g2o lib
find_package(G2O REQUIRED)
# find_package(G2O)
IF(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIR})
    message("G2O lib is found:" ${G2O_INCLUDE_DIR})
ENDIF(G2O_FOUND)

find_package(CSparse)

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIR})

# Lapack for Ceres Solver
find_package(LAPACK)
if (LAPACK_FOUND)
    include_directories(${LAPACK_INCLUDE_DIRS})
endif()

# CXSparse for Ceres Solver
find_package(CXSparse)
if (CXSPARSE_FOUND)
    include_directories(${CXSPARSE_INCLUDE_DIRS})
endif()

# SuiteSparse for Ceres Solver
find_package(SuiteSparse)
if (SUITESPARSE_FOUND)
    include_directories(${SUITESPARSE_INCLUDE_DIRS})
endif()

SET(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core
        g2o_interface
        g2o_solver_cholmod
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse
        )


include_directories(include)
include_directories(
        ${CSPARSE_INCLUDE_DIR}
        /usr/local/include/g2o/solvers/csparse
        )


include(cmake/eigen.cmake)
# include(cmake/g2o.cmake)

find_package(Python3 COMPONENTS Development NumPy)

aux_source_directory(./src SRCS)
add_executable(graph_slam ${SRCS})


target_include_directories(graph_slam PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${Ceres_INCLUDE_DIR} ${LAPACK_INCLUDE_DIRS}
${CXSPARSE_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS})
target_link_libraries(graph_slam Python3::Python Python3::NumPy ${G2O_LIBS} ${CSPARSE_LIBRARY} 
${CERES_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${CXSPARSE_LIBRARIES}
        ${SUITESPARSE_LIBRARIES})