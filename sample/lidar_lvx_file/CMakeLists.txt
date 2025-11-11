cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_lvx_sample)
add_executable(${DEMO_NAME} main.cpp lvx_file.cpp)

# 👇 Añadir estas dos líneas
find_library(GPIODCXX_LIB gpiodcxx REQUIRED)
find_library(GPIOD_LIB gpiod REQUIRED)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        ${GPIODCXX_LIB}
        ${GPIOD_LIB}
        )