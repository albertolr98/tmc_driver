cmake_minimum_required(VERSION 3.10)
project(tmc_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# 1. Definimos los archivos fuente DE LA LIBRERÍA (Sin el main.cpp)
set(TMC_LIB_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/spi_bus.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tmc5160.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tmc_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tmc_spi_registry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/TMC-API/tmc/ic/TMC5160/TMC5160.c
)

# 2. CREAMOS LA LIBRERÍA COMPARTIDA (Esto es lo que le faltaba a ow_hardware)
add_library(tmc_driver_lib SHARED ${TMC_LIB_SOURCES})

# Configuración de includes para la librería
target_compile_definitions(tmc_driver_lib PUBLIC TMC5160_IC_CACHE_COUNT=4)
target_include_directories(tmc_driver_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    third_party/TMC-API
    third_party/TMC-API/tmc
    third_party/TMC-API/tmc/helpers
    third_party/TMC-API/tmc/ic
    third_party/TMC-API/tmc/ic/TMC5160
)

# Enlazamos libgpiod a la librería
find_library(GPIOD_LIB gpiod)
if(NOT GPIOD_LIB)
  message(FATAL_ERROR "libgpiod not found, install with: sudo apt install libgpiod-dev")
endif()
target_link_libraries(tmc_driver_lib PRIVATE ${GPIOD_LIB})

# 3. (Opcional) Mantenemos tu ejecutable de test, pero ahora usa la librería
add_executable(test_drivers src/main.cpp)
target_link_libraries(test_drivers PRIVATE tmc_driver_lib) # Enlaza con la librería que acabamos de crear

# 4. INSTALACIÓN Y EXPORTACIÓN (Vital para que ow_hardware la encuentre)

# Instalar la librería
install(TARGETS tmc_driver_lib
  EXPORT export_tmc_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instalar el ejecutable de test
install(TARGETS test_drivers
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar cabeceras
install(DIRECTORY include/
  DESTINATION include
)

# 5. Exportar dependencias para otros paquetes
ament_export_targets(export_tmc_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(tmc_driver_lib) # <--- ¡ESTO ARREGLA EL ERROR DE SÍMBOLO!

ament_package()