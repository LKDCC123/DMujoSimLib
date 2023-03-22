cmake_minimum_required(VERSION 3.0)
project(7P2_Robot_Sim)

set(ROOT_PATH .)

include_directories( # 库头文件
    
)

link_directories(
    "SimulationLib/Mujocolib/bin"
)

aux_source_directory("SimulationLib" Dir_Sim)
aux_source_directory("SimulationLib/CRemotelib" Dir_Sim)
aux_source_directory("SimulationLib/Mujocolib" Dir_Sim)
aux_source_directory("SimulationLib/Mujocolib/include" Dir_Sim)
add_executable(Sim_process BHR7P2RobSim.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameCpp2C.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameLogger.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameSaveLog.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameSwitch.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/FullDoFIK.cpp
    BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/RobotState.cpp
    ${Dir_Sim} ${Dir_Eigen} ${Dir_ChzFm} ${Dir_DccLib} ${Dir_Lib}
)

target_compile_definitions(Sim_process PUBLIC -DMAX_EXT_API_CONNECTIONS=255 -DNON_MATLAB_PARSING -DDO_NOT_USE_SHARED_MEMORY -D_CRT_SECURE_NO_WARNINGS)
target_compile_options(Sim_process PUBLIC /nologo /Ox /O2 /Ot /W3 /WX- /Gm- /EHsc /MD /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR /Gd /openmp- /Gd /analyze- /errorReport:prompt /openmp- /EHsc /arch:AVX /I../include /Fe../bin/)
target_link_libraries(Sim_process PUBLIC glfw3.lib mujoco200.lib) 

file(COPY "${ROOT_PATH}/SimulationLib/Mujocolib/bin/mjkey.txt" DESTINATION "..")
file(COPY "${ROOT_PATH}/SimulationLib/Mujocolib/bin/glfw3.dll" DESTINATION "Release") 
file(COPY "${ROOT_PATH}/SimulationLib/Mujocolib/bin/mujoco200.dll" DESTINATION "Release") 
file(COPY "${ROOT_PATH}/SimulationLib/Mujocolib/bin/mujoco200nogl.dll" DESTINATION "Release") 