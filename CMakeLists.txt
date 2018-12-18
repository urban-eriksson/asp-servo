cmake_minimum_required(VERSION 3.9)

project(libasp_servo VERSION 0.0.1 DESCRIPTION "ASP robot servo API" LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(libsoem REQUIRED )

add_library(asp_servo SHARED
    src/asp_servo.cpp src/servo.cpp
)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(asp_servo 
    PUBLIC
       $<INSTALL_INTERFACE:include>
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE 
       ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(asp_servo soem pthread rt tinyxml2)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS asp_servo 
    EXPORT asp_servo-targets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
) 

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT asp_servo-targets
    FILE 
        asp_servoTargets.cmake
    NAMESPACE 
        asp_servo::
    DESTINATION 
        ${CMAKE_INSTALL_LIBDIR}/cmake/asp_servo
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
       ${CMAKE_CURRENT_BINARY_DIR}/asp_servoConfigVersion.cmake
    VERSION 
       ${PROJECT_VERSION}
    COMPATIBILITY 
       AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/asp_servoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/asp_servoConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/asp_servo
)

