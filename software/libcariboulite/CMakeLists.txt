cmake_minimum_required(VERSION 3.15)
project(cariboulite)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Bring the headers
include_directories(./ ./include ../kernel)
include_directories(${PROJECT_SOURCE_DIR}/src)

# ------------------------------------
# MAIN - Source files for main library
# ------------------------------------
set(SOURCES_LIB src/cariboulite_setup.c src/cariboulite_radios.c)

# Add internal project dependencies
add_subdirectory(src/caribou_fpga EXCLUDE_FROM_ALL)
add_subdirectory(src/at86rf215 EXCLUDE_FROM_ALL)
add_subdirectory(src/latticeice40 EXCLUDE_FROM_ALL)
add_subdirectory(src/io_utils EXCLUDE_FROM_ALL)
add_subdirectory(src/cariboulite_config EXCLUDE_FROM_ALL)
add_subdirectory(src/cariboulite_eeprom EXCLUDE_FROM_ALL)
add_subdirectory(src/zf_log EXCLUDE_FROM_ALL)

# Create the library LibCaribouLite
add_library(cariboulite STATIC ${SOURCES_LIB})
target_link_libraries(cariboulite
                                    caribou_fpga
                                    at86rf215
                                    latticeice40
                                    io_utils
                                    cariboulite_config
                                    cariboulite_eeprom
                                    zf_log
                                                                        )

target_include_directories(cariboulite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-Wall -Wextra -Wno-unused-variable -Wno-missing-braces)

# create the Soapy shared object
find_package(SoapySDR CONFIG)
if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

if(CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3")
    endif()
    #Thread support enabled (not the same as -lpthread)
    list(APPEND SOAPY_CARIBOULITE_LIBRARIES)
    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter -Wno-missing-field-initializers -Wno-parentheses -Wno-psabi)
endif(CMAKE_COMPILER_IS_GNUCXX)

SOAPY_SDR_MODULE_UTIL(
        TARGET SoapyMySdr
    SOURCES 
        src/soapy_api/SoapyCariboulite.cpp 
        src/soapy_api/Cariboulite.cpp
        src/soapy_api/Cariboulite.hpp
        src/soapy_api/CaribouliteStream.cpp
        src/soapy_api/CaribouliteSession.cpp
    LIBRARIES cariboulite
)


# ------------
# INSTALLATION
# ------------
# Set the location for library installation -- i.e., /usr/lib in this case
# not really necessary in this example. Use "sudo make install" to apply
#install(TARGETS cariboulite DESTINATION /usr/lib)
