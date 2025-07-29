cmake_minimum_required( VERSION 3.11 )
cmake_policy(SET CMP0048 NEW)

project( "CIPster on Linux" )

include(ExternalProject)

set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++17, ON -> -std=gnu++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)


set( CIPSTER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ )

set( USER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/get_flat_angles
    CACHE PATH "Location of user specific include file (cipster_user_conf.h)"
    )

if( CMAKE_BUILD_TYPE STREQUAL Debug )
    add_definitions( -DCIPSTER_WITH_TRACES -DCIPSTER_TRACE_LEVEL=15 )
    set( TRACE_SPEC "-DCIPster_TRACES=ON" )
endif()

add_definitions( -std=c++17 )

# PREFIX is for ExternalProject_Add, and tells where to build CIPster as a sub project:
# below our current out of tree build directory.
set( PREFIX ${CMAKE_CURRENT_BINARY_DIR}/build-CIPster )

##############################################################
## finding the ifm3d lib.
##############################################################
find_package(ifm3d 1.6.12 CONFIG
  REQUIRED )

# build CIPster as a nested project, the result of which is libeip.a
# in directory ${PREFIX}
ExternalProject_Add( eip
    PREFIX ${PREFIX}
    SOURCE_DIR ${CIPSTER_DIR}/source
    CONFIGURE_COMMAND
        ${CMAKE_COMMAND}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX=${PREFIX}
        -DCMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}
        -DCMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DUSER_INCLUDE_DIR=${USER_INCLUDE_DIR}
        ${TRACE_SPEC}       # empty for non Debug CMAKE_BUILD_TYPE
        <SOURCE_DIR>
    BUILD_COMMAND make

    INSTALL_COMMAND make install
    )

#message( "TRACE_SPEC=${TRACE_SPEC}" )


# These variables would typically be found by a CMake find EIP module, but set
# them manually here
set( EIP_INCLUDE_DIR ${CIPSTER_DIR}/source/src )
set( EIP_LIBRARIES   ${PREFIX}/libeip.a )


if( NOT APPLE )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_COMPILE_OPTIONS_VISIBILITY}hidden" )
endif()
if( NOT APPLE )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_COMPILE_OPTIONS_VISIBILITY_INLINES_HIDDEN}" )
endif()


include_directories(
    .
    ${EIP_INCLUDE_DIR}
    ${USER_INCLUDE_DIR}
    )

add_executable( flat_angles
    main.cc
    get_flat_angles/flat_angles.cc
    )
target_link_libraries( flat_angles
    ${EIP_LIBRARIES}
    ifm3d::ifm3d
    )
add_dependencies( flat_angles eip )

