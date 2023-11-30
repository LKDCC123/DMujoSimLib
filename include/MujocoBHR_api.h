#pragma once
#ifndef MUJICOBHR_LIB_H
#define MUJICOBHR_LIB_H
#ifdef MUJICOBHR_LIB_CPP
#define Extern 
#else
#define Extern extern
#endif

//-------------------------------- include -----------------------------------------------
#include "mjxmacro.h"
#include "uitools.h" // including "mujoco.h" (dynamix) & "glfw3.h" (visualization)
#include "string.h"
#include "stdio.h"
#include <thread>
#include <mutex>
#include <chrono>
#include "DataType.h"

//-------------------------------- defines -----------------------------------------------
struct st_Force {
    int nBodyNum;
    double dForce[3];
    double dTiming[2];
    int nKey;
};

//-------------------------------- global -----------------------------------------------_
// OpenGL rendering and UI
Extern GLFWwindow* MJwindow;

// UI settings not contained in MuJoCo structures
Extern struct
{
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 1;
    int ui1 = 1;
    int help = 0;               
    int info = 0;               
    int profiler = 0;           
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 0;

    // simulation
    int run = 1;
    int key = 0;
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
}settings;

void fnvMujocoSimuInit(int nPreDefMod, const char* ctpModName);
void fnvMujocoSimuLoop(
    int nJointNum,
    double _dJointsInitPos[], 
    double dptJointsPosition[], 
    double dptJointsVelocity[], 
    int nIMUNum,
    double dRotMat[][9],
    Dcc::MUJOCO_FILES::st_IMU dIMU[],
    int nFSNum,
    double dptFootFT[][6],
    double dptFSDirection[][6],
    double dptCmdJointsPosition[],
    double _dJointsDirection[],
    int nMotorMod,
    int nForceNum,
    st_Force stForce[],
    int nKey,
    int * nKpre,
    void (* pfLoop)(void)
    );
void fnvMujocoRenderLoop(void);
void fnvMujocoSimuEnd(void);

#undef Extern

#endif