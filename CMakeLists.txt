cmake_minimum_required(VERSION 3.0)
project(DMujoSim)

message("${CMAKE_PREFIX_PATH}")
set(BuldeAndInstall 1)
set(TestLib 0)

# BuldeAndInstall ------------------------------------------------------------------------------------------------------- 
if(${BuldeAndInstall})

find_package(DMujoMk)
find_package(DBase)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/bin
)

aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC_TEST)
aux_source_directory(${CMAKE_SOURCE_DIR}/include SRC_TEST)

add_executable(Test
    Test.cpp
    ${SRC_TEST}
)

target_compile_definitions(Test PUBLIC
    _CRT_SECURE_NO_WARNINGS
    NDEBUG
    UNICODE
    CONSOLE
)

target_compile_options(Test PUBLIC
    /nologo
)

target_link_libraries(Test PUBLIC
    glfw3.lib
    mujoco200.lib
)

file(COPY ${CMAKE_SOURCE_DIR}/bin/mjkey.txt DESTINATION ${PROJECT_BINARY_DIR}/Release)

# if install
include(config.cmake)

endif()

# TestLib -------------------------------------------------------------------------------------------------------
if(${TestLib})

find_package(DMujoMk)
find_package(DBase)
find_package(DMujoSim)

add_executable(TestLib
    Test.cpp
)

target_link_libraries(TestLib PUBLIC
    glfw3.lib
    mujoco200.lib
    DMujoSim.lib
)

endif()
