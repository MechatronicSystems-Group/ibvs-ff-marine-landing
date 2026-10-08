project(BargeController)

cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED ENV{GZ_DISTRO} OR NOT "$ENV{GZ_DISTRO}"  STREQUAL "harmonic")
    find_package(gz-transport NAMES gz-transport14 gz-transport13)
    find_package(gz-sim NAMES gz-sim9 gz-sim8)
    find_package(gz-sensors NAMES gz-sensors9 gz-sensors8)
    find_package(gz-plugin NAMES gz-plugin3 gz-plugin2 COMPONENTS register)
else()
    find_package(gz-transport NAMES gz-transport13)
    find_package(gz-sim NAMES gz-sim8)
    find_package(gz-sensors NAMES gz-sensors8)
    find_package(gz-plugin NAMES gz-plugin2 COMPONENTS register)
endif()

add_library(${PROJECT_NAME} SHARED
    BargeController.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC gz-sensors${gz-sensors_VERSION_MAJOR}::gz-sensors${gz-sensors_VERSION_MAJOR}
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
