#pragma once
#include "DMujoHeader.h"
#include <array>

_D_MUJOSIM_BEGIN

using double3 = std::array<double,3>;

struct st_IMU{
    double3 Ang, Omg, Acc;
    inline st_IMU(){
        this->Ang = {0,0,0};
        this->Omg = {0,0,0};
        this->Acc = {0,0,0};
    };
};

_D_MUJOSIM_END

