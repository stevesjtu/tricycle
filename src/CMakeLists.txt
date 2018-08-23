cmake_minimum_required (VERSION 2.6)
project (tricycle)

# Libraries building ######################################################
set(CMAKE_CXX_STANDARD 14)

include_directories(
		${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/eigen334
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
link_directories(${PROJECT_BINARY_DIR}/bin)


set(
    TRICYCLE_SRC
    triangle_mesh.cc
    contact_sensor.cc
)

aux_source_directory(./opcode/Ice ICE_SRC)
aux_source_directory(./opcode OPCODE_SRC)

add_library (tricycle SHARED ${TRICYCLE_SRC} ${ICE_SRC} ${OPCODE_SRC})