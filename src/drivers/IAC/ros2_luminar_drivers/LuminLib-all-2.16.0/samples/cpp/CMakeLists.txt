cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(LuminLibSample)

# If building independently (and not as part of the top level LuminLib),
# will need to set the path to LuminLib using LuminLib_DIR env var
if(NOT "${CMAKE_PROJECT_NAME}" STREQUAL "LuminLib" AND NOT "${CMAKE_PROJECT_NAME}" STREQUAL "EnVision")
    set(LuminLib_DIR "$ENV{LuminLib_DIR}" CACHE PATH "LuminLib cmake directory")
    find_package(LuminLib REQUIRED NO_DEFAULT_PATH)
endif()

add_executable(${PROJECT_NAME}
    LuminLibSample.cpp
    DataClientHelper.cpp
    ControlClientHelper.cpp
)

# C++14 compiler conformance strictly enforced
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14) #(11, 14, 17, 20)

target_sources(${PROJECT_NAME}
    PUBLIC
    ControlClientHelper.h
    DataClientHelper.h
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    LuminLib
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ../
    )

if(WIN32)
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

    get_WIN32_WINNT(ver)
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            -D_SCL_SECURE_NO_WARNINGS=1
            -D_CRT_SECURE_NO_WARNINGS=1
            -D_WIN32_WINNT=${ver})

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        target_compile_options(${PROJECT_NAME}
            PRIVATE
                /MP         # Parallel building
                /wd4251     # Class needs to have dll-interface (from Qt, needed as of 5.11.0)
                /Zo         # Enhance optimized debugging
        )
    endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
endif(WIN32)
