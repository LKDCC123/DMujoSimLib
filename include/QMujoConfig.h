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

extern st_Camera stCamera;

#endif