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
target_compile_definitions(${PROJECT_NAME} PRIVATE TMC5160_IC_CACHE_COUNT=4)

find_library(GPIOD_LIB gpiod)
if (NOT GPIOD_LIB)
  message(FATAL_ERROR "libgpiod not found, install with: sudo apt install libgpiod-dev")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${GPIOD_LIB})
