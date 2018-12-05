cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ImageRecording)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-fPIC -Wall -Wextra -pedantic -Wno-import)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    include
    /usr/include
    /usr/local/include
)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_library(SavingCalibrationDataClass SHARED
    src/SavingCalibrationDataClass.cpp
)
target_link_libraries(SavingCalibrationDataClass
    ${OpenCV_LIBS}
)
add_library(CameraCalibrationClass SHARED
    src/CameraCalibrationClass.cpp
)
target_link_libraries(CameraCalibrationClass
    ${OpenCV_LIBS}
    SavingCalibrationDataClass
)
add_executable(cameraCalibration
    src/main.cpp
)

target_link_libraries(cameraCalibration
    ${OpenCV_LIBS}
    SavingCalibrationDataClass
    CameraCalibrationClass
)
