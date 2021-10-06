cmake_minimum_required (VERSION 3.5.0)
project (UDPToCSV)
add_executable(
    UDPToCSV
    UDPToCSV.c
    lumDataPacketIndexer.c
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../
  )

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -std=c99 -Wc++-compat -Wall -Wextra -Werror -pedantic -Wwrite-strings -Wno-error=unused-parameter -Wswitch-default -Wswitch-enum -Wfloat-equal -Wno-error=format -Wshadow -Wno-error=format-signedness -Wdouble-promotion -Wduplicated-cond -Wlogical-op -Wcast-qual -Wfloat-conversion -Wstrict-prototypes -Wold-style-definition -Warray-bounds=2 -Wduplicated-branches -Wshift-overflow=2
    )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options(${PROJECT_NAME} PRIVATE
    /permissive /W4 /w14640 /w14545 /w34242 /w34254 /w34287 /w44263 /w44265 /w44296 /w44311 /w44826 /we4289 /w14546 /w14547 /w14549 /w14555 /w14619 /w14905 /w14906 /w14928
    )
endif()

if(WIN32)
  target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()
