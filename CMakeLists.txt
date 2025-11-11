cmake_minimum_required(VERSION 3.10)
project(TMCDriver C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  include
  third_party/TMC-API
  third_party/TMC-API/tmc
  third_party/TMC-API/tmc/helpers
  third_party/TMC-API/tmc/ic
  third_party/TMC-API/tmc/ic/TMC5160
)

file(GLOB SRC src/*.cpp)
set(TMC_API_SOURCES
  third_party/TMC-API/tmc/ic/TMC5160/TMC5160.c
)

add_executable(${PROJECT_NAME} ${SRC} ${TMC_API_SOURCES})
