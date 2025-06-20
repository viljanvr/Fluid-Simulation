cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0072 NEW)

# This is now a C++ project
project(FluidSim CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to enable Address Sanitizer
option(ENABLE_ASAN "Enable Address Sanitizer" OFF)

# Set the source directory
set(SRC_DIR code)

# Define source files (now .cpp files)
set(SOURCES
        ${SRC_DIR}/demo.cpp
        ${SRC_DIR}/solver.cpp
        ${SRC_DIR}/SolidObjects/RectangleObstacle.cpp
        ${SRC_DIR}/SolidObjects/SolidBoundary.cpp
        ${SRC_DIR}/Object.cpp
        ${SRC_DIR}/linearSolver.cpp
        ${SRC_DIR}/ConjGradMatrix.cpp
        ${SRC_DIR}/ScenePresets.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# HAVE_CONFIG_H needs to be defined for GFX to work properly
target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_CONFIG_H)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        include/SolidObjects
        include/Colormaps
)

if(WIN32)
  target_include_directories(${PROJECT_NAME} PRIVATE include/Utils/windows)
elseif(UNIX)
  target_include_directories(${PROJECT_NAME} PRIVATE include/Utils/linux)
endif()

# OS-specific OpenGL/GLUT configuration
if(APPLE)
  target_link_libraries(${PROJECT_NAME} "-framework OpenGL" "-framework GLUT" m)
  target_include_directories(${PROJECT_NAME} PRIVATE "/opt/homebrew/include")
elseif(UNIX)  # Linux / WSL
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  target_include_directories(${PROJECT_NAME} PRIVATE
          ${OPENGL_INCLUDE_DIR}
          ${GLUT_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME} OpenGL::GL OpenGL::GLU GLUT::GLUT m)
endif()

# Add Address Sanitizer flags if enabled
message(STATUS "Address Sanitizer enabled")
target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
target_link_libraries(${PROJECT_NAME} -fsanitize=address)
