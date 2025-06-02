cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0072 NEW)
project(FluidSim C)

set(CMAKE_C_STANDARD 99)

# Set the source directory
set(SRC_DIR code)

# Define source files
set(SOURCES
        ${SRC_DIR}/demo.c
        ${SRC_DIR}/solver.c
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# OS-specific OpenGL/GLUT configuration
if(APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL" "-framework GLUT" m)

elseif(UNIX)  # Linux / WSL
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} OpenGL::GL OpenGL::GLU GLUT::GLUT m)
endif()