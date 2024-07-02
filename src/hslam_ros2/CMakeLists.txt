cmake_minimum_required(VERSION 3.5)
project(hslam_ros2)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -march=native -fno-tree-vectorize "
#"${SSE_FLAGS} -O2 -g -std=c++14 -march=native "
)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native -std=gnu++14 -fno-tree-vectorize")


# set(CMAKE_CXX_FLAGS
#    "${SSE_FLAGS} -O3 -g -std=c++14 -march=native -Wno-deprecated -Wno-deprecated-declarations" #-fopenmp is added if g2o is compiled with openmp capabilities
# #   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
# )
# #SET CXX FLAGS to use march antive and gcc
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -std=gnu++14")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -std=gnu++14")

# if (MSVC)
#      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
# endif (MSVC)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)


set(FSLAM_PATH $ENV{WORKSPACE_DIR}/src/FSLAM)
#set(FSLAM_LIBRARY_PATH $ENV{WORKSPACE_DIR}/src/FSLAM/lib)
set(FSLAM_LIBRARY_PATH $ENV{WORKSPACE_DIR}/src/FSLAM/build/lib)

# set values from environment variables
if (NOT FSLAM_PATH)
    set(FSLAM_PATH $ENV{FSLAM_PATH})
endif()
if (NOT FSLAM_LIBRARY_PATH)
    set(FSLAM_LIBRARY_PATH $ENV{FSLAM_LIBRARY_PATH})
    if (NOT FSLAM_LIBRARY_PATH AND FSLAM_PATH)
        set(FSLAM_LIBRARY_PATH "${FSLAM_PATH}/build/lib")
    endif()
endif()

if (FSLAM_PATH)
    # Manually set path to FSLAM source folder
    message("FSLAM_PATH set, trying to find library.")
    message("---- EXPECTING FSLAM sources at\n\"${FSLAM_PATH}\"")
    set(FSLAM_INCLUDE_DIRS "${FSLAM_PATH}/src" "${FSLAM_PATH}/Thirdparty/Sophus" "${FSLAM_PATH}/Thirdparty/g2o" "${FSLAM_PATH}/Thirdparty/DBow3" "${FSLAM_PATH}/Thirdparty/opencv-3.4.6")
    message("---- LOOKING FOR FSLAM library at\n\"${FSLAM_LIBRARY_PATH}\"")
    find_library(FSLAM_LIBRARY HSLAM_lib ${FSLAM_LIBRARY_PATH})
# else()
#     # Detect FSLAM install (also catkin devel folder)
#     message("FSLAM_PATH not set yet, trying to find installed fslam headers and library.")
#     find_path(FSLAM_INCLUDE_DIRS FSLAM)
#     if (FSLAM_INCLUDE_DIRS)
#         set(FSLAM_INCLUDE_DIRS "${FSLAM_INCLUDE_DIRS}/FSLAM")
#         message("---- FOUND FSLAM headers at \"${FSLAM_INCLUDE_DIRS}\"")
#     endif()
#     find_library(FSLAM_LIBRARY fslam)
endif()


if (NOT FSLAM_INCLUDE_DIRS)
    message(FATAL_ERROR "FSLAM headers not found and/or FSLAM_PATH not set.")
endif()
if (NOT FSLAM_LIBRARY)
    message(FATAL_ERROR "FSLAM library not found and/or FSLAM_LIBRARY_PATH not set.")
endif()

message("---- Found FSLAM library at \"${FSLAM_LIBRARY}\"")


set(CMAKE_MODULE_PATH $ENV{WORKSPACE_DIR}/src/hslam_ros2/cmake)
message("this is CMAKE_MODULE_PATH \"${CMAKE_MODULE_PATH}\"")
set(OpenCV_DIR $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/share/OpenCV)
set(Pangolin_DIR $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake)
set(DBoW3_DIR $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake/DBoW3)
set(G2O_DIR $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake/g2o)


find_package(Pangolin 0.2 REQUIRED PATHS $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV 3.4.6 REQUIRED PATHS $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(DBoW3 REQUIRED)
message("\n\n---- FOUND DBoW3 Headers at\n\"${DBoW3_INCLUDE_DIRS}\"")
message("---- FOUND DBoW3 Libs at\n\"${DBoW3_LIBRARIES}\"")


find_package(G2O COMPONENTS)
#G2O not found troubleshooting
# message("-----G2O troubleshooting-----")
# message("The G2O directory is : ${G2O_DIR}" )
# message("The G2O include  directory is : ${G2O_INCLUDE_DIR}" )
# message("The G2O libraries is : ${G2O_LIBS}" )
#find_package(G2O REQUIRED COMPONENTS PATHS $ENV{WORKSPACE_DIR}/src/FSLAM/Thirdparty/CompiledLibs/lib/cmake/g2o)
message ("The following were consodered for g2o: ${G2O_CONSIDERED_CONFIGS}")
message("\n\n---- FOUND G2O Headers at\n\"${G2O_INCLUDE_DIR}\"")
message("---- FOUND G2O Libs at\n\"${G2O_LIBS}\"")

#set(CSPARSE_DIR "/usr/include/suitesparse")
find_package(SuiteParse REQUIRED)
message("\n\n---- FOUND CSPARSE at\n\"${CSPARSE_INCLUDE_DIR}\"")
message("\n\n---- FOUND CSPARSE LIBS at\n\"${CSPARSE_LIBRARY}\"")
message("\n\n---- FOUND CHOLMOD at\n\"${CHOLMOD_INCLUDE_DIR}\"")
message("\n\n---- FOUND CHOLMOD LIBS at\n\"${CHOLMOD_LIBRARIES}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread system REQUIRED) 


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()


set(LIBS
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  ${Pangolin_LIBRARIES}
  ${DBoW3_LIB_DIR}/libDBoW3.so
  ${BOOST_THREAD_LIBRARY}
  boost_system
  cxsparse
  cholmod
  fmt
)




INCLUDE_DIRECTORIES(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${FSLAM_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${DBoW3_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

link_directories(
  ${G2O_INCLUDE_DIR}
  ${DBoW3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)





add_executable(hslam_live src/hslam_main.cpp)

# Set the runtime search path to include the necessary directories
set_target_properties(hslam_live PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${FSLAM_LIBRARY_PATH}:${CMAKE_INSTALL_RPATH}"
)
ament_target_dependencies(hslam_live rclcpp std_msgs geometry_msgs nav_msgs pcl_ros sensor_msgs pcl_conversions)


target_link_libraries(hslam_live
    ${FSLAM_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${G2O_LIBS}
    ${DBoW3_LIB_DIR}/libDBoW3.so
    ${CSPARSE_LIBRARY}
    ${CHOLMOD_LIBRARIES}
    ${Boost_LIBRARIES} #
   )


install(TARGETS hslam_live
  DESTINATION lib/${PROJECT_NAME})
 

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
