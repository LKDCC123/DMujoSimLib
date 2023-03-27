# - Config file for the DMujoSim package
# It defines the following variables
#  DMujoSim_INCLUDE_DIR - include directories for DMujoSim
#  DMujoSim_SRC_DIR     - src     directories for DMujoSim
#  DMujoSim_LIB_DIR     - lib     directories for DMujoSim
 
# Compute paths
if(1)
    set(DMujoSim_INCLUDE_DIR "D:/_pkg/DPackages/DMujoSim/include")
endif()
if(1)
    set(DMujoSim_LIB_DIR     "D:/_pkg/DPackages/DMujoSim/lib"    )
endif()
if(0)
    set(DMujoSim_SRC_DIR     "D:/_pkg/DPackages/DMujoSim/src"    )
endif()

# set paths
message("-- [DMujoSim]: Package found!")
if(DEFINED DMujoSim_INCLUDE_DIR)
    include_directories(${DMujoSim_INCLUDE_DIR})
    message("-- [DMujoSim]: Include ${DMujoSim_INCLUDE_DIR}")
endif()
if(DEFINED DMujoSim_LIB_DIR)
    link_directories(${DMujoSim_LIB_DIR})
    # link_libraries(DMujoSim.lib)
    # link_libraries(glfw3.lib)
    # link_libraries(mujoco200.lib)
    message("-- [DMujoSim]: Link ${DMujoSim_LIB_DIR}")
endif()
if(DEFINED DMujoSim_SRC_DIR)
    aux_source_directory(${DMujoSim_SRC_DIR} DMujoSim_FILES)
    message("-- [DMujoSim]: Can use DMujoSim_SRC_FILES")
endif()
