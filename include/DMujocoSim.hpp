// DMujocoSim.hpp
// simulation lib based on mujoco and dcc MMaker
// 20230322 dcc <3120195094@bit.edu.cn>

#pragma once
#ifndef DMUJOCOSIM_HPP
#define DMUJOCOSIM_HPP
#include <thread>
#include <conio.h>
#include <QMujoConfig.h>
#include "DMujoHeader.h"
#include "MujocoBHR_api.h"
#include <DataType.h>

_D_MUJOSIM_BEGIN

#define __FloatingBaseNum 7 // quaternion and position of the floating base
#define __MaxFrcNum 10

struct st_SimInit {
    double JointsDirection[__MaxJointNum]; // init the joints' direction
    double ForceSDirection[__MaxFSNum][6]; // init the force sensors' direction
    double JointsInitSpc[__MaxJointNum + __FloatingBaseNum]; // init the state of the robot
};

struct st_SimIO {
    struct { // input to the Mujoco simulation
        double JointsPos[__MaxJointNum];
    }Cmd; 
    struct { // output from the Mujoco simulation
        double RotMat[__MaxIMUNum][9]; // rotmat is received from mujoco
        st_IMU IMU[__MaxIMUNum]; // decoded rotation: [rot_x, rot_y, rot_z]
        double JointPos[__MaxJointNum]; // sensed real joints position
        double JointVel[__MaxJointNum]; // sensed real joints velocity
        double FS[__MaxFSNum][6]; // sensed force sensor [fx, fy, fz, tx, ty ,tz]
    }Sen; 
};

static COORD OutChar;
static HANDLE hOut;
// create a new thread to get key and for users' print
void fnvHmi(
    int *nKey,
    int *nKeyFetched,
    void (*pfPrint)(void)) {
    OutChar.X = 0;
    OutChar.Y = 0;
    hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    system("cls");
    while(!settings.exitrequest) {
        SetConsoleCursorPosition(hOut, OutChar);
        if(*nKeyFetched) *nKey = *nKeyFetched = 0; // if key is fetched by the user, clear the key
        if(_kbhit()) *nKey = _getch(); // if key pressed, set key
        if(pfPrint != NULL) pfPrint(); // run the users print function
        Sleep(30);
        if(*nKey == '\\') system("cls"); // clear the print
        if(*nKey == ' ') settings.exitrequest = 1; // quit simulation
    }
}

#define _MMk    c_MMaker::

class c_MujoSim:public c_MMaker {
public:
    int m_nKProg = 0;
    c_MujoSim(const char * cptModelName, double dTimeStep,  int nMotorMod, int nIfGravity):c_MMaker(cptModelName, dTimeStep, nMotorMod, nIfGravity) {
        this->m_cptModName = cptModelName;
        this->m_nKey = 0;
        this->m_nKeyFetched = 0;
    }

    ~c_MujoSim() {

    }

    // make .xml file only
    bool MakeModel() {
        this->_MMk fnvWriteXML(); // wirte the .xml file for the simualtion
        this->_MMk fnvDisp(); // display the number of bodies and joints
        return true;
    }

    // using external model
    bool Init(st_SimInit * stptSimInit, st_SimIO * stptSimIO, const char * cptModelPath) { // transfer data: pointer set
        this->MakeModel();
        char cptFullPath[__MaxStrLen];
        strcpy(cptFullPath, cptModelPath);
        strcat(cptFullPath, "/");
        strcat(cptFullPath, this->m_cptModName); // get the full path of the external model
        strcat(cptFullPath, ".xml"); 
        _STD cout << "Reading model from: " << cptFullPath << _STD endl;
        _STD cout << "Mujoco is ready!!" << _STD endl;
        _STD cout << "Press enter to continue..." << _STD endl;
        getchar();
        fnvMujocoSimuInit(1, cptFullPath); // set model
        this->m_SimIO = stptSimIO;
        this->m_SimIOInit = stptSimInit;
        this->m_nInitFlag = true;
        this->m_nForceNum = 0;
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

    // run simulation, press space to quit
    bool Run(void (*pfLoop)(void), void (*pfPrint)(void)) {
        _STD thread ThPrint(
            fnvHmi, 
            &this->m_nKey,
            &this->m_nKeyFetched,
            pfPrint); // start the hmi in a new thread
        if(this->m_nInitFlag) {
            _STD thread MujocoSimThread(
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
                this->m_SimIOInit->ForceSDirection,
                this->m_SimIO->Cmd.JointsPos,
                this->m_SimIOInit->JointsDirection,
                this->_MMk m_nMotorMod,
                this->m_nForceNum,
                this->m_stForce,
                this->m_nKey,
                &this->m_nKProg,
                pfLoop); // start simulation in a new thread
            while (!glfwWindowShouldClose(MJwindow) && !settings.exitrequest) fnvMujocoRenderLoop(); // render loop in the current thread
            // end simulation
            fnvMujocoSimuEnd();
            MujocoSimThread.join(); 
            ThPrint.join();
            return true;
        }
        else return false;
    }

    // add push force on certain body at the right timing 
    bool AddForce(char * cptBodyName, double3 dFrc, double dSpan, double dTiming) {
        for(int i = 0; i < this->_MMk m_nBodyNum; i++) {
            if(strcmp(cptBodyName, this->_MMk m_stBodysInfo[i].bodyName) == 0) {
                this->m_stForce->nBodyNum = i;
                for(int j = 0; j < 3; j++) this->m_stForce->dForce[j] = dFrc[j];
                this->m_stForce->dTiming[0] = dTiming, this->m_stForce->dTiming[1] = dTiming + dSpan;
                this->m_stForce->nKey = 0;
                this->m_nForceNum++;
                return true;
            }
        }
        _STD cout << "Wrong body name!!" << _STD endl;
        return false;
    }

    // add push force on certain body triggered by a pressed key // Todo
    // bool AddForce(char * cptBodyName, double3 dFrc, double dSpan, int nKeyTriger) {
    //     for(int i = 0; i < this->_MMk m_nBodyNum; i++) {
    //         if(strcmp(cptBodyName, this->_MMk m_stBodysInfo[i].bodyName) == 0) {
    //             this->m_stForce->nBodyNum = i;
    //             for(int j = 0; j < 3; j++) this->m_stForce->dForce[j] = dFrc[j];
    //             this->m_stForce->dTiming[0] = 0.0, this->m_stForce->dTiming[1] = dSpan;
    //             this->m_stForce->nKey = nKeyTriger;
    //             this->m_nForceNum++;
    //             return true;
    //         }
    //     }
    //     _STD cout << "Wrong body name!!" << _STD endl;
    //     return false;
    // }

    // fetch the key in users function and clear the key
    const int &GetKey() {
        this->m_nKeyFetched = 1;
        return this->m_nKey;
    }

    void setCamera(double focus[3], double zoom, double azimuth, double elevation, int type, int TrackingID) {
        memcpy(stCamera.focus, focus, sizeof(stCamera.focus));
        stCamera.zoom = zoom;
        stCamera.azi = azimuth;
        stCamera.ele = elevation;
        stCamera.type = type;
        stCamera.TrackingID = TrackingID;
    }

private:
    int m_nInitFlag;
    int m_nErrorFlag;
    int m_nKey;
    int m_nKeyFetched; // if the key command is fetched and only fetched once by external users code, clear the key command
    const char * m_cptModName;
    double m_dTimeStep;
    st_SimIO * m_SimIO;
    st_SimInit * m_SimIOInit;
    st_Force m_stForce[__MaxFrcNum];
    int m_nForceNum;
    bool fnbClearKey() {
        this->m_nKey = 0;
        return true;
    }
};

_D_MUJOSIM_END

#endif