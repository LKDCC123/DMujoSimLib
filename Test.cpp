// test the mujoco simulation
#include <DMujocoSim.hpp>
#include <DBase.hpp>

_D_USING_MUJO

st_SimInit stSimInit = { 
    { 
      1.0, -1.0, -1.0, // waist yaw, left arm, right arm
      1.0, 1.0, -1.0, -1.0, -1.0, 1.0, // left leg
      1.0, 1.0, -1.0, -1.0, -1.0, 1.0 // right leg
    },
    { // force sensor direction
      -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, // L foot
      -1.0, -1.0, -1.0, -1.0, 1.0, -1.0  // R foot
    }, 
    { // let robot stand with a little bended knee
      0.0, 0.0, 0.72, // trunk position
      0.0, 0.0, 0.0, 0.0, // trunk quaternion
      0.0, 0.0, 0.0, // waist yaw, left arm, right arm
      0.0, 0.0, __D2R(12.5), __D2R(-25.0), __D2R(12.5), 0.0, // left leg
      0.0, 0.0, __D2R(12.5), __D2R(-25.0), __D2R(12.5), 0.0 // right leg 
    }
};

st_SimIO stSimIO;

c_MujoSim cMujoSim("Test", 0.004, __PosCon, __Gravity);

void ConLoop() {
    stSimIO.Cmd.JointsPos[0] = 0.0;
    stSimIO.Cmd.JointsPos[1] = 0.0;
    stSimIO.Cmd.JointsPos[2] = 0.0;
    stSimIO.Cmd.JointsPos[3] = 0.0;
    stSimIO.Cmd.JointsPos[4] = 0.0;
    stSimIO.Cmd.JointsPos[5] = __D2R(12.5);
    stSimIO.Cmd.JointsPos[6] = __D2R(-25.0);
    stSimIO.Cmd.JointsPos[7] = __D2R(12.5);
    stSimIO.Cmd.JointsPos[8] = 0.0;
    stSimIO.Cmd.JointsPos[9] = 0.0;
    stSimIO.Cmd.JointsPos[10] = 0.0;
    stSimIO.Cmd.JointsPos[11] = __D2R(12.5);
    stSimIO.Cmd.JointsPos[12] = __D2R(-25.0);
    stSimIO.Cmd.JointsPos[13] = __D2R(12.5);
    stSimIO.Cmd.JointsPos[14] = 0.0;
    // _STD cout << __R2D(stSimIO.Sen.IMU[0][0]) << ", " << __R2D(stSimIO.Sen.IMU[0][1]) << _STD endl;
    // int nF = 4;
    // _STD cout << stSimIO.Sen.FS[0][nF] << ", " << stSimIO.Sen.FS[1][nF] << _STD endl; // dcc here
    // _STD cout << cMujoSim.m_nKProg << _STD endl;
}

void fnvPrint() {
  int hehe = cMujoSim.GetKey();
  printf("\t%c\n", hehe);
  printf("Omg: %7.3f, %7.3f, %7.3f\t", stSimIO.Sen.IMU[0].Omg[0], stSimIO.Sen.IMU[0].Omg[1], stSimIO.Sen.IMU[0].Omg[2]);
  printf("Acc: %7.3f, %7.3f, %7.3f", stSimIO.Sen.IMU[0].Acc[0], stSimIO.Sen.IMU[0].Acc[1], stSimIO.Sen.IMU[0].Acc[2]);
}

void main() {
    // add units            body name      inb name            mass    inerx    inery   inerz   if geom length  joints name                                             joints axis                             body to joint               inb to joint
    cMujoSim.fnvAddBase   ("midbody",     {0.0, 0.0, 0.72},   0.0001, {0.0001,  0.0001,  0.0001},       0.16                                                                                                                                                       );
    cMujoSim.fnvAddPin    ("uppbody",     "midbody",          19.500, {0.6100,  0.3800,  0.3000}, 1,    -0.5,   "waist_yaw",                                            {0, 0, 1},                            { 0.042,  0.000, -0.296},   { 0.000,  0.000,  0.000} );
    cMujoSim.fnvAddPin    ("larm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "left_arm",                                             {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000,  0.272,  0.115} );
    cMujoSim.fnvAddPin    ("rarm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "right_arm",                                            {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000, -0.272,  0.115} );
    cMujoSim.fnvAddBall   ("lthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "left_leg_1",   "left_leg_2",   "left_leg_3",           {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014, -0.015,  0.101},   { 0.000,  0.080,  0.000} );
    cMujoSim.fnvAddPin    ("lshank",      "lthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "left_leg_4",                                           {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014, -0.015, -0.219} );
    cMujoSim.fnvAddGimbal ("lankle",      "lshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "left_l3eg_5",   "left_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMujoSim.fnvAddFoot   ("lfoot",       "lankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_lforcesensor", "lforcesonsor", "ltorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035,  0.020, -0.015} );
    cMujoSim.fnvAddBall   ("rthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "right_leg_1",  "right_leg_2",  "right_leg_3",          {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014,  0.015,  0.101},   { 0.000, -0.080,  0.000} );
    cMujoSim.fnvAddPin    ("rshank",      "rthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "right_leg_4",                                          {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014,  0.015, -0.219} );
    cMujoSim.fnvAddGimbal ("rankle",      "rshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "right_leg_5",  "right_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMujoSim.fnvAddFoot   ("rfoot",       "rankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_rforcesensor", "rforcesonsor", "rtorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035, -0.020,  -0.015} );
    cMujoSim.fnvAddIMU    ("site_imu",    "uppbody",                                                                                                                                                                                      { 0.042,  0.000, -0.296} );
    
    // exclude contacts
    cMujoSim.fnvExContact("midbody", "uppbody");
    cMujoSim.fnvExContact("midbody", "larm");
    cMujoSim.fnvExContact("midbody", "rarm");
    cMujoSim.fnvExContact("midbody", "lthigh");
    cMujoSim.fnvExContact("midbody", "lshank");
    cMujoSim.fnvExContact("midbody", "lankle");
    cMujoSim.fnvExContact("midbody", "rthigh");
    cMujoSim.fnvExContact("midbody", "rshank");
    cMujoSim.fnvExContact("midbody", "rankle");
    cMujoSim.fnvExContact("uppbody", "larm");
    cMujoSim.fnvExContact("uppbody", "rarm");
    cMujoSim.fnvExContact("uppbody", "lthigh");
    cMujoSim.fnvExContact("uppbody", "lshank");
    cMujoSim.fnvExContact("uppbody", "lankle");
    cMujoSim.fnvExContact("uppbody", "rthigh");
    cMujoSim.fnvExContact("uppbody", "rshank");
    cMujoSim.fnvExContact("uppbody", "rankle");
    cMujoSim.fnvExContact("larm", "rarm");
    cMujoSim.fnvExContact("larm", "lthigh");
    cMujoSim.fnvExContact("larm", "lshank");
    cMujoSim.fnvExContact("larm", "lankle");
    cMujoSim.fnvExContact("larm", "rthigh");
    cMujoSim.fnvExContact("larm", "rshank");
    cMujoSim.fnvExContact("larm", "rankle");
    cMujoSim.fnvExContact("rarm", "lthigh");
    cMujoSim.fnvExContact("rarm", "lshank");
    cMujoSim.fnvExContact("rarm", "lankle");
    cMujoSim.fnvExContact("rarm", "rthigh");
    cMujoSim.fnvExContact("rarm", "rshank");
    cMujoSim.fnvExContact("rarm", "rankle");
    cMujoSim.fnvExContact("lthigh", "lshank");
    cMujoSim.fnvExContact("lthigh", "lankle");
    cMujoSim.fnvExContact("lthigh", "rthigh");
    cMujoSim.fnvExContact("lthigh", "rshank");
    cMujoSim.fnvExContact("lthigh", "rankle");
    cMujoSim.fnvExContact("lshank", "lankle");
    cMujoSim.fnvExContact("lshank", "rthigh");
    cMujoSim.fnvExContact("lshank", "rshank");
    cMujoSim.fnvExContact("lshank", "rankle");
    cMujoSim.fnvExContact("lankle", "rthigh");
    cMujoSim.fnvExContact("lankle", "rshank");
    cMujoSim.fnvExContact("lankle", "rankle");
    cMujoSim.fnvExContact("rthigh", "rshank");
    cMujoSim.fnvExContact("rthigh", "rankle");
    cMujoSim.fnvExContact("rshank", "rankle");

    cMujoSim.fnvBuildBlock(0.0, {1.0, 1.0, 0.1}, {1.0, 1.0, 0.05}, {0.1, 0.1, 0.1});
    cMujoSim.fnvBuildBlock(0.0, {1.0, 1.0, 0.1}, {2.0, 2.0, 0.15}, {0.0, 0.0, 0.0});

    cMujoSim.Init(&stSimInit, &stSimIO);

    cMujoSim.Run(ConLoop, fnvPrint);
}
