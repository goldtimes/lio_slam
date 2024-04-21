list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# opencv
find_package(OpenCV REQUIRED)
# # find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# link_libraries(${CERES_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
# include_directories( ${CERES_INCLUDE_DIRS})
link_directories(${${CERES_LIBRARIES}})

find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

## third part
# 其他thirdparty下的内容
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include(FetchContent)

FetchContent_Declare(
        tessil 
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tessil-src)

if (NOT tessil_POPULATED)
    set(BUILD_TESTING OFF)
    FetchContent_Populate(tessil)

    add_library(robin_map INTERFACE)
    add_library(tsl::robin_map ALIAS robin_map)

    target_include_directories(robin_map INTERFACE
            "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")

    list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
    target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")

    if (MSVC)
        target_sources(robin_map INTERFACE
                "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/tsl-robin-map.natvis>")
    endif ()
endif ()

set(third_party_libs
    ${catkin_LIBRARIES}
    ${g2o_libs}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    glog 
    gflags
    ${yaml-cpp_LIBRARIES}
    robin_map
    yaml-cpp
)