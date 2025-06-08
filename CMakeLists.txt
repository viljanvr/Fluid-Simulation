cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0072 NEW)

# This is now a C++ project
project(FluidSim CXX)

include_directories(include)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the source directory
set(SRC_DIR code)

# Define source files (now .cpp files)
set(SOURCES
        ${SRC_DIR}/demo.cpp
        ${SRC_DIR}/solver.cpp
        ${SRC_DIR}/SolidObject.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_CONFIG_H)

if (WIN32)
  target_include_directories(${PROJECT_NAME} PRIVATE "include/windows")
elseif (UNIX)
  target_include_directories(${PROJECT_NAME} PRIVATE "include/linux")
endif()

# OS-specific OpenGL/GLUT configuration
if(APPLE)
  target_link_libraries(${PROJECT_NAME} "-framework OpenGL" "-framework GLUT" m)
  target_include_directories(${PROJECT_NAME} PRIVATE "/opt/homebrew/include")
elseif(UNIX)  # Linux / WSL
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  target_include_directories(${PROJECT_NAME} PRIVATE ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} OpenGL::GL OpenGL::GLU GLUT::GLUT m)
endif()
