cmake_minimum_required(VERSION 2.8)
project(relocal)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++11")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#Eigen
include_directories("/usr/include/eigen3-3")

# Opencv
find_package(OpenCV 3.1)
include_directories(${OpenCV_INCLUDE_DIRS})

#Super4PCS
set(Super4PCS_DIR "/usr/local/lib/cmake/")
find_package(Super4PCS REQUIRED)
include_directories(${Super4PCS_INCLUDE_DIR})

# NanoFLann 
set(NANOFLANN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/sparseicp/include)
include_directories(${NANOFLANN_INCLUDE_DIR})

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/include )

add_library(relocal src/Converter.cc
					src/Frame.cc
					src/FrameDrawer.cc
                    src/Initializer.cc
					src/KeyFrame.cc
					src/KeyFrameDatabase.cc
					src/LocalMapping.cc
					src/LoopClosing.cc
					src/Map.cc
					src/MapPoint.cc 
					src/MapDrawer.cc
					src/Optimizer.cc
                    src/ORBextractor.cc
                    src/ORBmatcher.cc
					src/PnPsolver.cc
					src/Sim3Solver.cc
					src/Tracking.cc
					src/Viewer.cc
                    src/System.cc
					src/PoseSolver.cc
					src/Registrating.cc)

target_link_libraries(relocal
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${EIGEN3_LIBS}
# ${PCL_LIBRARIES}
${OpenCV_LIBS} 
${Pangolin_LIBRARIES}
${Super4PCS_LIBRARIES})

add_executable(main main.cc)
target_link_libraries(main relocal)
