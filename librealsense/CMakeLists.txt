#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSensePCLExample)

add_executable(rs-tracker rs-tracker.cpp)
set_property(TARGET rs-tracker PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-tracker ${DEPENDENCIES})
if(NOT glfw3_DIR)
    target_include_directories(rs-tracker PUBLIC "${CMAKE_SOURCE_DIR}/third-party/glfw/include")
    target_link_directories(rs-tracker PUBLIC "${CMAKE_BINARY_DIR}/third-party/glfw/src/${CMAKE_BUILD_TYPE}")
endif()
set_target_properties (rs-tracker PROPERTIES
    FOLDER "Examples/PCL"
)

foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
    endif(${flag_var} MATCHES "/MT")
endforeach(flag_var)

install(
    TARGETS

    rs-tracker

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
