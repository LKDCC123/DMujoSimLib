// DMujoHeader.h

#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <DModelMaker.hpp>
// #include "MujocoBHR_api.h"

#ifndef __PosCon 
#define __PosCon 0
#endif
#ifndef __TrqCon 
#define __TrqCon 1
#endif
#ifndef __ZeroGravity 
#define __ZeroGravity 0
#endif
#ifndef __Gravity 
#define __Gravity 1
#endif

#define _D_MUJOSIM_BEGIN namespace Dcc { namespace MUJOCO_FILES {
#define _D_MUJOSIM_END }}
#define _D_MUJO ::Dcc::MUJOCO_FILES::
#define _D_USING_MUJO using namespace Dcc::MUJOCO_FILES;