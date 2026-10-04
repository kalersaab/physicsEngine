cmake_minimum_required(VERSION 3.20)

project(physicsEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(kphysics

        src/math/Vec3.cpp
        src/math/Mat3.cpp
        src/core/RigidBody.cpp
        src/core/PhysicsWorld.cpp
        src/math/Quaternion.cpp
)

target_include_directories(kphysics
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(physicsEngine
        main.cpp
)

target_link_libraries(physicsEngine
        PRIVATE
        kphysics
)