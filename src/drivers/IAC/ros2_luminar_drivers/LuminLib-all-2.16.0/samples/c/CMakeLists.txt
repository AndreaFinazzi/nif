cmake_minimum_required (VERSION 3.5.0)
project (UDPToCSV)
add_executable( 
    UDPToCSV
    UDPToCSV.c
    lumDataPacketIndexer.c
)

# C++14 compiler conformance strictly enforced
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14) #(11, 14, 17, 20)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../LumNet-includes
    )

if(WIN32)
  target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()
