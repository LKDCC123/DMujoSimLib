// DMujocoSim.hpp
// simulation lib based on mujoco and dcc MMaker
// 20230322 dcc <3120195094@bit.edu.cn>

#pragma once
#ifndef DMUJOCOSIM_HPP
#define DMUJOCOSIM_HPP
#include "DMujoHeader.h"

_D_MUJOSIM_BEGIN

#define __FloatingBaseNum 7 // quaternion and position of the floating base

struct st_SimInit {
    double JointsDirection[__MaxJointNum]; // init the joints' direction
    double JointsInitSpc[__MaxJointNum + __FloatingBaseNum]; // init the state of the robot
};

struct st_SimIO {
    struct { // input to the Mujoco simulation
        double JointsPos[__MaxJointNum];
    }Cmd; 
    struct { // output from the Mujoco simulation
        double RotMat[__MaxIMUNum][9]; // rotmat is received from mujoco
        double IMU[__MaxIMUNum][3]; // decoded rotation: [rot_x, rot_y, rot_z]
        double JointPos[__MaxJointNum]; // sensed real joints position
        double JointVel[__MaxJointNum]; // sensed real joints velocity
        double FS[__MaxFSNum][6]; // sensed force sensor [fx, fy, fz, tx, ty ,tz]
    }Sen; 
};

#define _MMk    c_MMaker::

class c_MujoSim:public c_MMaker {
public:
    int m_nKProg = 0;
    c_MujoSim(const char * cptModelName, double dTimeStep, int nIfGravity):c_MMaker(cptModelName, dTimeStep, nIfGravity) {
        this->m_cptModName = cptModelName;
    }

    ~c_MujoSim() {

    }

    // using external model
    bool Init(st_SimInit * stptSimInit, st_SimIO * stptSimIO, const char * cptModelPath) { // transfer data: pointer set
        this->_MMk fnvWriteXML(); // wirte the .xml file for the simualtion
        this->_MMk fnvDisp(); // display the number of bodies and joints
        char cptFullPath[__MaxStrLen];
        strcpy(cptFullPath, cptModelPath);
        strcat(cptFullPath, "/");
        strcat(cptFullPath, this->m_cptModName); // get the full path of the external model
        strcat(cptFullPath, ".xml"); 
        _STD cout << "Reading model from: " << cptFullPath << _STD endl;
        fnvMujocoSimuInit(1, cptFullPath); // set model
        this->m_SimIO = stptSimIO;
        this->m_SimIOInit = stptSimInit;
        this->m_nInitFlag = true;
        _STD cout << "Mujoco is ready!!" << _STD endl;
        return true;
    }

    // using interal model
    bool Init(st_SimInit * stptSimInit, st_SimIO * stptSimIO) { // transfer data: pointer set
        if(!this->_MMk m_nBodyNum) {
            _STD cout << "Have not build the model yet!!" << _STD endl;
            _STD cout << "Using 'fnvAddBase', 'fnvAddPin', 'fnvAddGimbal', 'fnvAddBall', 'fnvAddIMU', 'fnvAddFoot', 'fnvExContact', to build the model." << _STD endl;
            this->m_nInitFlag = false;
            return false;
        }
        this->Init(stptSimInit, stptSimIO, "./");
        return true;
    }

    // run simulation
    bool Run(void (*pfLoop)(void)) {
        if(this->m_nInitFlag) {
            std::thread MujocoSimThread(
                fnvMujocoSimuLoop,
                this->m_nJointNum,
                this->m_SimIOInit->JointsInitSpc,
                this->m_SimIO->Sen.JointPos,
                this->m_SimIO->Sen.JointVel,
                this->m_nIMUNum,
                this->m_SimIO->Sen.RotMat,
                this->m_SimIO->Sen.IMU,
                this->m_nFSNum,
                this->m_SimIO->Sen.FS,
                this->m_SimIO->Cmd.JointsPos,
                this->m_SimIOInit->JointsDirection,
                &this->m_nKProg,
                pfLoop  
                ); // start simulation in a background thread
            while (!glfwWindowShouldClose(MJwindow) && !settings.exitrequest) fnvMujocoRenderLoop(); // render loop in the current thread
            // end simulation
            fnvMujocoSimuEnd();
            MujocoSimThread.join(); 
            return true;
        }
        else return false;
    }

private:
    int m_nInitFlag;
    int m_nErrorFlag;
    const char * m_cptModName;
    double m_dTimeStep;
    st_SimIO * m_SimIO;
    st_SimInit * m_SimIOInit;
};

_D_MUJOSIM_END

#endif