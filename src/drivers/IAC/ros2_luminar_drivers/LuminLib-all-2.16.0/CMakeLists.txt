cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
include(GenerateExportHeader)

project(LuminLib
    VERSION 2.16.0
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(LUMINLIB_TYPE "all" CACHE STRING "Desired library type - all, control only, data only")
set_property(CACHE LUMINLIB_TYPE PROPERTY STRINGS all control data)

STRING(COMPARE EQUAL ${LUMINLIB_TYPE} "all" LIB_ALL)
STRING(COMPARE EQUAL ${LUMINLIB_TYPE} "control" LIB_CONTROL)
STRING(COMPARE EQUAL ${LUMINLIB_TYPE} "data" LIB_DATA)

if(LIB_CONTROL)
    message(STATUS "BUILDING CONTROL CLIENT ONLY")
endif(LIB_CONTROL)

if(LIB_DATA)
    message(STATUS "BUILDING DATA CLIENT ONLY")
endif(LIB_DATA)

option(BUILD_SAMPLE_PROJECTS "Enable building the sample projects" OFF)
if(BUILD_SAMPLE_PROJECTS)
    add_subdirectory(samples/c)
    add_subdirectory(samples/cpp)
    add_subdirectory(LumNet-includes/Examples/StatusPacketParser)
endif(BUILD_SAMPLE_PROJECTS)

set(${PROJECT_NAME}_VERSION_UNDERSCORE ${PROJECT_VERSION_MAJOR}_${PROJECT_VERSION_MINOR}_${PROJECT_VERSION_PATCH})

# Enable assert on release builds
option(LUMINAR_ENABLE_ASSERT_RELEASE "Enable asserts on release builds." OFF)
if(LUMINAR_ENABLE_ASSERT_RELEASE)
    string( REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    string( REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELINFO "${CMAKE_CXX_FLAGS_RELINFO}")
endif(LUMINAR_ENABLE_ASSERT_RELEASE)

if(MSVC)
    if(MSVC_VERSION EQUAL 1900)
        # VS 2015
        set(VS_VER vc140)
    elseif(MSVC_VERSION GREATER_EQUAL 1910 AND MSVC_VERSION LESS_EQUAL 1919 )
        # VS 2017
        set(VS_VER vc141)
    elseif(MSVC_VERSION GREATER_EQUAL 1920 AND MSVC_VERSION LESS_EQUAL 1929 )
        # VS 2019
        set(VS_VER vc142)
    else()
        message(FATAL_ERROR "Unsupported MSVC version: `${MSVC_VERSION}`" )
    endif()
endif()

# Offer the user the choice of overriding the installation directories
set(LUMINLIB_INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(LUMINLIB_INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(LUMINLIB_INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(LUMINLIB_INSTALL_SOURCE_DIR src CACHE PATH "Installation directory for source files")
set(LUMINLIB_CONTROL_INSTALL_INCLUDE_DIR Control_Client/include CACHE PATH "Installation directory for header files")
set(LUMINLIB_CONTROL_INSTALL_SOURCE_DIR Control_Client/src CACHE PATH "Installation directory for source files")
set(LUMINLIB_DATA_INSTALL_INCLUDE_DIR Data_Client/include CACHE PATH "Installation directory for header files")
set(LUMINLIB_DATA_INSTALL_SOURCE_DIR Data_Client/src CACHE PATH "Installation directory for source files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR cmake)
else()
    set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

if(NOT IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet)
    message( FATAL_ERROR "LumNet-includes does not contain the required `LumNet` directory. \nDid you run `git submodule update --init --recursive` ?")
endif()

if(NOT Boost_FOUND)
    set(BOOST_ROOT "$ENV{BOOST_ROOT}" CACHE PATH "Boost installation root")
    set(Boost_USE_STATIC_LIBS ON)
    find_package(Boost 1.58 REQUIRED COMPONENTS system date_time regex)
endif()

if(NOT glm_FOUND AND (LIB_DATA OR LIB_ALL))
    set(glm_DIR "$ENV{glm_DIR}" CACHE PATH "glm cmake directory")
    find_package(glm REQUIRED)
endif(NOT glm_FOUND AND (LIB_DATA OR LIB_ALL))

if(UNIX)
    find_package(Threads REQUIRED)
    set(EXTRA_PLATFORM_LIBS Threads::Threads)
endif()

macro(get_WIN32_WINNT version)
    if (CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REGEX MATCH "^([0-9]+).([0-9])" ver ${ver})
        string(REGEX MATCH "^([0-9]+)" verMajor ${ver})
        # Check for Windows 10, b/c we'll need to convert to hex 'A'.
        if ("${verMajor}" MATCHES "10")
            set(verMajor "A")
            string(REGEX REPLACE "^([0-9]+)" ${verMajor} ver ${ver})
        endif ("${verMajor}" MATCHES "10")
        # Remove all remaining '.' characters.
        string(REPLACE "." "" ver ${ver})
        # Prepend each digit with a zero.
        string(REGEX REPLACE "([0-9A-Z])" "0\\1" ver ${ver})
        set(${version} "0x${ver}")
    endif(CMAKE_SYSTEM_VERSION)
endmacro(get_WIN32_WINNT)

add_library(${PROJECT_NAME} STATIC)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        # C++14 compiler conformance strictly enforced
        CXX_STANDARD_REQUIRED ON
        # (11, 14, 17, 20)
        CXX_STANDARD 14
)

generate_export_header(${PROJECT_NAME}
    EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/include/luminlib_export.h
)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
target_compile_definitions(${PROJECT_NAME} PUBLIC
    ${PROJECT_NAME_UPPER}_STATIC_DEFINE)

set (LUMNET_FILE_PREFIX "external")

if(LUMINAR_INTERNAL)
    include("${CMAKE_CURRENT_SOURCE_DIR}/LuminLib-Internal.cmake" OPTIONAL RESULT_VARIABLE INTERNAL_FOUND)

    if(NOT INTERNAL_FOUND)
        MESSAGE( WARNING "Internal features for LuminLib requested, but could not be loaded. Continuing as normal build.")
    endif()
endif()

set( CONTROL_GENERATED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/CommandConstructors.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/CommandPayloads.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/CommandPredicates.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/ModelHCommandClientSubscriber.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/ModelHControllerCommands.h
)
set( CONTROL_GENERATED_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/src/${LUMNET_FILE_PREFIX}/CommandConstructors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/src/${LUMNET_FILE_PREFIX}/CommandPayloads.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/src/${LUMNET_FILE_PREFIX}/CommandPredicates.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/src/${LUMNET_FILE_PREFIX}/ModelHControllerCommands.cpp
)

set (CSC_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX})

set (CONTROL_HEADERS
    ${CONTROL_GENERATED_HEADERS}

    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/CommandLumNetDefinitions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/DiscoveryResponse.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/${LUMNET_FILE_PREFIX}/ModelHController.h

    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/ResponseServer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/SensorConnectionSubscriber.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/SensorConnectionStateManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/SensorSocketService.h

    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/CommandLumNetRequest.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/CommandLumNetResponse.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/CommandSizes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/CommandUtilities.h

    ${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include/CommonDefinitions.h

    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Discovery.h

    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/Payloads.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Discovery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/LidarData.h
)


set (CONTROL_SOURCES
    ${CONTROL_GENERATED_SOURCES}

    Control_Client/src/CommandUtilities.c
    Control_Client/src/CommandLumNetRequest.c
    Control_Client/src/CommandLumNetResponse.c
    Control_Client/src/CommandSizes.c

    Control_Client/src/${LUMNET_FILE_PREFIX}/ModelHController.cpp
    Control_Client/src/ResponseServer.cpp
    Control_Client/src/SensorSocketService.cpp
    Control_Client/src/SensorConnectionStateManager.cpp
    Control_Client/src/${LUMNET_FILE_PREFIX}/DiscoveryResponse.cpp
)

include("${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/data_model/GenerateControlFiles.cmake" OPTIONAL )

set (DATA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/${LUMNET_FILE_PREFIX})

set (DATA_SOURCES
    Data_Client/src/BitOperations.cpp
    Data_Client/src/ModelHClient.cpp
    Data_Client/src/ModelHDataPacketParser.cpp
    Data_Client/src/${LUMNET_FILE_PREFIX}/ModelHDistributor.cpp
    Data_Client/src/${LUMNET_FILE_PREFIX}/ModelHReturnIterator.cpp
    Data_Client/src/ModelHSubscriptionManager.cpp
)

set (DATA_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/BinaryHeader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/BitOperations.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/Conversions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/LidarReturn.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/ModelHClient.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/ModelHDistributor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/ModelHDataPacketParser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/${LUMNET_FILE_PREFIX}/ModelHReturnIterator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/ModelHSubscriber.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/ModelHSubscriptionManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/PagedMemoryPool.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include/SensorPose.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Discovery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/LidarData.h
)

set (GLOBAL_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/luminlib_export.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ModelHCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/SensorUID.h
)

set (GLOBAL_SOURCES
    src/SensorUID.cpp
)

set (LEGACY_LUMNET_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet/Discovery.h
)

set (LUMNET_HEADERS
    ${LEGACY_LUMNET_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Lidar/Data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Lidar/Detector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Lidar/Laser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Lidar/Scanner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Cmd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Dsc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Gen.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Ptp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net/Status.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Sys/Factory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Sys/Health.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Sys/Ident.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Sys/Mode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Lidar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Net.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps/Sys.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/AddrOps.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/Payloads.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command/VerifyOps.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Common/ErrorTypeID.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Common/PayloadTypeID.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Common/ShiftAndMask.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Status/AddrOps.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Status/Payloads.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Discovery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/LidarData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet/Status.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LegacyLumNet.h
    ${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes/LumNet.h
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
  )

if(LIB_CONTROL OR LIB_ALL)
    target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Control_Client/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes>

        ${CSC_GENERATED_INCLUDE_DIR}
    )

endif(LIB_CONTROL OR LIB_ALL)
if(LIB_DATA OR LIB_ALL)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Data_Client/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LumNet-includes>

    ${DATA_INCLUDE_DIR}
  PRIVATE
    ${GLM_INCLUDE_DIRS}
  )
endif(LIB_DATA OR LIB_ALL)

target_sources(${PROJECT_NAME}
  PRIVATE
    ${GLOBAL_HEADERS} ${GLOBAL_SOURCES} ${LUMNET_HEADERS}
  )
if(LIB_CONTROL OR LIB_ALL)
target_sources(${PROJECT_NAME}
  PRIVATE
    ${CONTROL_HEADERS} ${CONTROL_SOURCES}
  )
endif(LIB_CONTROL OR LIB_ALL)
if(LIB_DATA OR LIB_ALL)
target_sources(${PROJECT_NAME}
  PRIVATE
    ${DATA_HEADERS} ${DATA_SOURCES}
  )
endif(LIB_DATA OR LIB_ALL)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Boost::system
    Boost::date_time
    Boost::regex
    ${EXTRA_PLATFORM_LIBS}
)

if(MSVC)
    get_WIN32_WINNT(ver)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE
            _CRT_SECURE_NO_WARNINGS
            _WIN32_WINNT=${ver}
    )
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            /MP         # Parallel building
            /wd4251     # Class needs to have dll-interface (from Qt, needed as of 5.11.0)
            /Zo         # Enhance optimized debugging
    )

    # Debugging helpers
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY
        VS_STARTUP_PROJECT ${PROJECT_NAME})
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(WIN32)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
            RELEASE_POSTFIX        "-${LUMINLIB_TYPE}-${VS_VER}-mt-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
            RELWITHDEBINFO_POSTFIX "-${LUMINLIB_TYPE}-${VS_VER}-mt-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
            DEBUG_POSTFIX          "-${LUMINLIB_TYPE}-${VS_VER}-mt-gd-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
    )
elseif(UNIX)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
            RELEASE_POSTFIX        "-${LUMINLIB_TYPE}-mt-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
            RELWITHDEBINFO_POSTFIX "-${LUMINLIB_TYPE}-mt-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
            DEBUG_POSTFIX          "-${LUMINLIB_TYPE}-mt-gd-${${PROJECT_NAME}_VERSION_UNDERSCORE}"
    )
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND WIN32)
     set(CMAKE_INSTALL_PREFIX "C:/${PROJECT_NAME}" CACHE PATH "default install path" FORCE )
endif()

# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_NAME} FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})
set(EXPORT_TARGETS ${PROJECT_NAME} CACHE INTERNAL "export targets")

file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONFIG_INCLUDE_DIRS "${${PROJECT_NAME}_SOURCE_DIR}/include" )
configure_file(${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for the install tree
set(CONFIG_INCLUDE_DIRS "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

# If an install() target does not specify COMPONENT, assume project name
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION ${LUMINLIB_INSTALL_INCLUDE_DIR})
install(DIRECTORY src/ DESTINATION ${LUMINLIB_INSTALL_SOURCE_DIR})
if(LIB_CONTROL OR LIB_ALL)
    install(DIRECTORY Control_Client/include/ DESTINATION ${LUMINLIB_CONTROL_INSTALL_INCLUDE_DIR}
      PATTERN "internal" EXCLUDE
      )

    install(DIRECTORY Control_Client/src/ DESTINATION ${LUMINLIB_CONTROL_INSTALL_SOURCE_DIR}
      PATTERN "internal" EXCLUDE
      )

    install(DIRECTORY LumNet-includes/LegacyLumNet DESTINATION LumNet-includes)
    install(FILES LumNet-includes/LegacyLumNet.h DESTINATION LumNet-includes)
endif(LIB_CONTROL OR LIB_ALL)
if(LIB_DATA OR LIB_ALL)
    install(DIRECTORY Data_Client/include/ DESTINATION ${LUMINLIB_DATA_INSTALL_INCLUDE_DIR}
      PATTERN "internal" EXCLUDE
      )
    install(DIRECTORY Data_Client/src/ DESTINATION ${LUMINLIB_DATA_INSTALL_SOURCE_DIR}
      PATTERN "internal" EXCLUDE
      )

    get_filename_component(glm_INSTALL_DIR ${GLM_INCLUDE_DIRS} DIRECTORY)
    install(DIRECTORY
        ${glm_INSTALL_DIR}/
        DESTINATION "glm/installed")
endif(LIB_DATA OR LIB_ALL)
install(DIRECTORY
    LumNet-includes/LumNet
    DESTINATION LumNet-includes)
install(FILES
    LumNet-includes/LumNet.h
    DESTINATION LumNet-includes)
install(FILES
    CMakeLists.txt
    ${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}ConfigVersion.cmake.in
    DESTINATION ".")
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    DESTINATION ".")
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/licenses
    DESTINATION ".")
if(LIB_ALL)
    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/samples
        DESTINATION ".")
    install(DIRECTORY
        LumNet-includes/Examples
        DESTINATION LumNet-includes)
endif(LIB_ALL)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${LUMINLIB_TYPE})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PROJECT_NAME}-${LUMINLIB_TYPE})
set(CPACK_PACKAGE_VENDOR "Luminar Technologies")
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
set(CPACK_GENERATOR "ZIP")
include(CPack)
