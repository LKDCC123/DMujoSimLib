#pragma once
#ifndef QMUJOCONFIG_HPP
#define QMUJOCONFIG_HPP

struct st_Camera{
    double focus[3]; 
	double azi; 
	double ele; 
	double zoom; 

	int TrackingID;
	int type;
}; //by qhx
extern st_Camera q_stCamera;

typedef enum _VisualFlag            // flags enabling model element visualization
{
	VIS_CONVEXHULL = 0,           // mesh convex hull
	VIS_TEXTURE,                  // textures
	VIS_JOINT,                    // joints
	VIS_ACTUATOR,                 // actuators
	VIS_CAMERA,                   // cameras
	VIS_LIGHT,                    // lights
	VIS_TENDON,                   // tendons
	VIS_RANGEFINDER,              // rangefinder sensors
	VIS_CONSTRAINT,               // point constraints
	VIS_INERTIA,                  // equivalent inertia boxes
	VIS_SCLINERTIA,               // scale equivalent inertia boxes with mass
	VIS_PERTFORCE,                // perturbation force
	VIS_PERTOBJ,                  // perturbation object
	VIS_CONTACTPOINT,             // contact points
	VIS_CONTACTFORCE,             // contact force
	VIS_CONTACTSPLIT,             // split contact force into normal and tanget
	VIS_TRANSPARENT,              // make dynamic geoms more transparent
	VIS_AUTOCONNECT,              // auto connect joints and body coms
	VIS_COM,                      // center of mass
	VIS_SELECT,                   // selection point
	VIS_STATIC,                   // static bodies
	VIS_SKIN,                     // skin

	NumOfVISFLAG                    // number of visualization flags
} VisualFlag;
extern bool q_bVisualOptions[NumOfVISFLAG];

extern int q_iFontScale;
#endif