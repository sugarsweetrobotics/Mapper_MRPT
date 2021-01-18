cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Mapper_MRPT)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
include("${PROJECT_SOURCE_DIR}/cmake/utils.cmake")
set(PROJECT_VERSION 1.0.0 CACHE STRING "Mapper_MRPT version")
DISSECT_VERSION()
set(PROJECT_DESCRIPTION "Mapper RTC using MRPT")
set(PROJECT_VENDOR "ssr")
set(PROJECT_AUTHOR "ssr")
set(PROJECT_AUTHOR_SHORT "ssr")

# Add an "uninstall" target
CONFIGURE_FILE ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET (uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#option(BUILD_EXAMPLES "Build and install examples" OFF)
option(BUILD_DOCUMENTATION "Build the documentation" ON)
#option(BUILD_TESTS "Build the tests" OFF)
#option(BUILD_TOOLS "Build the tools" OFF)
option(BUILD_IDL "Build and install idl" ON)
option(BUILD_SOURCES "Build and install sources" OFF)

option(STATIC_LIBS "Build static libraries" OFF)
if(STATIC_LIBS)
    set(LIB_TYPE STATIC)
else(STATIC_LIBS)
    set(LIB_TYPE SHARED)
endif(STATIC_LIBS)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   # Mac OS X specific code
   SET(CMAKE_CXX_COMPILER "g++")
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


# Set up installation directories
set(BIN_INSTALL_DIR "components/bin")
set(LIB_INSTALL_DIR "components/lib")
set(INC_INSTALL_DIR
    "components/include/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")
set(SHARE_INSTALL_DIR
    "components/share/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")

# Get necessary dependency information
find_package(OpenRTM)
if(${OpenRTM_FOUND})
  MESSAGE(STATUS "OpenRTM configuration Found")
else(${OpenRTM_FOUND})
  message(STATUS "Use cmake/Modules/FindOpenRTM.cmake in the project")
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
  find_package(OpenRTM REQUIRED)
endif(${OpenRTM_FOUND})

# Universal settings

find_package(mrpt REQUIRED COMPONENTS obs poses maps slam config)

if (FALSE)
set(MRPT_INCLUDE_DIRS 
${mrpt_DIR}/include

${mrpt_DIR}/libs/nanoflann/include
${mrpt_DIR}/libs/nanoflann/include

${mrpt_DIR}/include/mrpt/config/include
${mrpt_DIR}/include/mrpt/containers/include
${mrpt_DIR}/include/mrpt/core/include
${mrpt_DIR}/include/mrpt/img/include
${mrpt_DIR}/include/mrpt/io/include
${mrpt_DIR}/include/mrpt/maps/include
${mrpt_DIR}/include/mrpt/math/include
${mrpt_DIR}/include/mrpt/obs/include
${mrpt_DIR}/include/mrpt/opengl/include
${mrpt_DIR}/include/mrpt/poses/include
${mrpt_DIR}/include/mrpt/rtti/include
${mrpt_DIR}/include/mrpt/serialization/include
${mrpt_DIR}/include/mrpt/slam/include
${mrpt_DIR}/include/mrpt/system/include
${mrpt_DIR}/include/mrpt/tfest/include
${mrpt_DIR}/include/mrpt/typemeta/include
${mrpt_DIR}/include/mrpt/util/include
)

set(MRPT_LIBRARY_DIRS
${mrpt_DIR}/lib
$
)

set(MRPT_LIBS
libmrpt-config216_msvc142_x64
libmrpt-containers216_msvc142_x64
libmrpt-core216_msvc142_x64
libmrpt-io216_msvc142_x64
libmrpt-maps216_msvc142_x64
libmrpt-math216_msvc142_x64
libmrpt-obs216_msvc142_x64
libmrpt-poses216_msvc142_x64
libmrpt-rtti216_msvc142_x64
libmrpt-serialization216_msvc142_x64
libmrpt-slam216_msvc142_x64
)


set(MRPT_DLIBS
libmrpt-config216_msvc142_x64
libmrpt-containers216_msvc142_x64
libmrpt-core216_msvc142_x64
libmrpt-io216_msvc142_x64
libmrpt-maps216_msvc142_x64
libmrpt-math216_msvc142_x64
libmrpt-obs216_msvc142_x64
libmrpt-poses216_msvc142_x64
libmrpt-rtti216_msvc142_x64
libmrpt-serialization216_msvc142_x64
libmrpt-slam216_msvc142_x64
)
# find_package(mrpt-slam REQUIRED)
endif()


#enable_testing()

# Subdirectories
add_subdirectory(cmake)
if(BUILD_DOCUMENTATION)
    add_subdirectory(doc)
endif(BUILD_DOCUMENTATION)

#if(BUILD_EXAMPLES)
#    add_subdirectory(examples)
#endif(BUILD_EXAMPLES)

if(BUILD_IDL)
    add_subdirectory(idl)
endif(BUILD_IDL)

add_subdirectory(include)
MAP_ADD_STR(headers  "include/" comp_hdrs)
add_subdirectory(src)

#if(BUILD_TESTS)
#    add_subdirectory(test)
#endif(BUILD_TESTS)

#if(BUILD_TOOLS)
#    add_subdirectory(tools)
#endif(BUILD_TOOLS)

if(BUILD_SOURCES)
    add_subdirectory(include)
    add_subdirectory(src)
endif(BUILD_SOURCES)

# Package creation
# By default, do not warn when built on machines using only VS Express:
IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
ENDIF()
include(InstallRequiredSystemLibraries)
set(PROJECT_EXECUTABLES ${PROJECT_NAME_LOWER}Comp
    "${PROJECT_NAME_LOWER}Comp")

set(cpack_options "${PROJECT_BINARY_DIR}/cpack_options.cmake")

configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    ${cpack_options} @ONLY)

set(CPACK_PROJECT_CONFIG_FILE ${cpack_options})
include(${CPACK_PROJECT_CONFIG_FILE})
include(CPack)


