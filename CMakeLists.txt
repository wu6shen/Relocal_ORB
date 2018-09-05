cmake_minimum_required(VERSION 2.8)
project(relocal)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++11 -mfpmath=sse -Wall -Wextra")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#Eigen
include_directories("/usr/include/eigen3-3")

# Opencv
find_package(OpenCV 3.1)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL
find_package(PCL 1.8.1 REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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
                    src/Relocal.cc src/Registration.cc)

target_link_libraries(relocal
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${OpenCV_LIBS} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES}
${PCL_REGISTRATION_LIBRARIES}
${Pangolin_LIBRARIES})

add_executable(main main.cc)
target_link_libraries(main relocal)
