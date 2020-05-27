cmake_minimum_required(VERSION 3.5.1)

project(steering VERSION 1.0)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
include(GNUInstallDirs)

add_library(steering SHARED
    src/steering_control.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${EIGEN_LIBRARIES})

set_target_properties(steering PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(steering PRIVATE includes)
target_include_directories(steering PRIVATE src)


install(TARGETS steering EXPORT steeringConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME})

install(DIRECTORY includes/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(EXPORT steeringConfig DESTINATION share/steering/cmake)


export(TARGETS ${PROJECT_NAME} FILE steeringConfig.cmake)
