#pragma once
#ifndef MUJICOBHR_LIB_H
#define MUJICOBHR_LIB_H
#ifdef MUJICOBHR_LIB_CPP
#define Extern 
#else
#define Extern extern
#endif

//-------------------------------- include -----------------------------------------------
#include "include/mjxmacro.h"
#include "include/uitools.h" // including "mujoco.h" (dynamix) & "glfw3.h" (visualization)
#include "string.h"
#include "stdio.h"
#include <thread>
#include <mutex>
#include <chrono>

//-------------------------------- global -----------------------------------------------
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
} settings;

void fnvMujocoSimuInit(int nPreDefMod, const char* ctpModName);
void fnvMujocoSimuLoop(void);
void fnvMujocoRenderLoop(void);
void fnvMujocoSimuEnd(void);

#undef Extern

#endif