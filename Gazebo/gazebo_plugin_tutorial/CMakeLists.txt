cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# Nombre del proyecto
project(GazeboPluginTutorial)

# Encontrar los paquetes necesarios
find_package(gazebo REQUIRED)

# Añadir directorios de inclusión
include_directories(${GAZEBO_INCLUDE_DIRS})

# Añadir directorios de linkeo
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Compilación de la librería compartida
add_library(hello_world SHARED pluguins/hello_world.cpp)
add_library(model_push SHARED pluguins/model_push.cpp)
add_library(FixedJointVelocity SHARED pluguins/FixedJointVelocity.cpp)

target_link_libraries(FixedJointVelocity ${GAZEBO_LIBRARIES})
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
# C++ estándar
set(CMAKE_CXX_STANDARD 11)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

