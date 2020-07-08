/*
A simple, lightweight but high performing PID library.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
*/

#include "PID.h"
#include <Arduino.h>
#include <streaming.h>

PID::PID(void) {
	_dT = 0.01;
	controllerType = cPID;

	maxOutput = 1;
	minOutput = 1;

	init();
}

PID::PID(uint8_t cType, float dT, float max, float min)
{
	_dT = dT;
	maxOutput = max;
	minOutput = min;

	controllerType = cType;

	init();
}

void PID::init() {
	_lastError = 0;
	_sumError = 0;
	_lastDterm = 0;
	_lastSetpoint = 0;
	intDeadband = 0;
	resetInt = false;
	setpoint = 0;
	_lastInput = 0;

	R = 0;

	v[0] = 0;	// Output filter state variables
	v[1] = 0;
}

float PID::calculate()
{
	float error = setpoint - input;

	_Pterm = K*error;

	float output = _Pterm;

	if (controllerType&cPI) {
		float errorDeadzone = 0;
		if (intDeadband > 0) {
			if (abs(error) < intDeadband) {
				errorDeadzone = 0;
				if (resetInt) _Iterm = 0;
			} else if (error < -intDeadband) {
				errorDeadzone = error + intDeadband;
			} else if (error > intDeadband) {
				errorDeadzone = error - intDeadband;
			}
		} else {
			errorDeadzone = error;
		}
		_Iterm += _if*errorDeadzone;
		if (_Iterm > maxOutput) {
			_Iterm = maxOutput;
		} else if (_Iterm < minOutput) {
			_Iterm = minOutput;
		}
		output += _Iterm;
	}

	if (controllerType&cPD) {
		// _Dterm = _lastDterm*_df1 + (error-_lastError)*_df2;
		_Dterm = _lastDterm*_df1 + (setpoint-_lastSetpoint)*R*_df2 - (input-_lastInput)*_df2;
		_lastInput = input;
		_lastDterm = _Dterm;
		_lastError = error;
		_lastSetpoint = setpoint;
		output += _Dterm;
	}

	if (controllerType&cLP) {
		output = outputFilter(output);
	}

	if (output > maxOutput) {
		output = maxOutput;
	} else if (output < minOutput) {
		output = minOutput;
	}

	// int16_t output2 = (int16_t) output;

	return output;
}

void PID::setParameters(float nK, float nTi, float nTd, float nN)
{
	K  = nK;
	Ti = nTi;
	Td = nTd;
	N  = nN;

	updateParameters();
}

void PID::updateParameters()
{
	_if = _dT/Ti;

	_df1 = Td/(Td+N*_dT);
	_df2 = Td*N/(Td+N*_dT);
}

void PID::reset() {
	_Iterm = 0;
	_Dterm = 0;
	_lastDterm = 0;
	_lastInput = 0;
}

void PID::resetITerm()
{
	_Iterm = 0;
}

void PID::resetDTerm()
{
	_Dterm = 0;
	_lastDterm = 0;
	_lastInput = 0;
}

// Second order Butterworth low pass filter
// Code obtained from http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=1000&frequencyLow=100&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
float PID::outputFilter(float x)
{
	// 100 Hz
	// v[0] = v[1];
	// v[1] = v[2];
	// v[2] = (6.745527388907e-2 * x)
		 // + ( -0.4128015981 * v[0])
		 // + (  1.1429805025 * v[1]);
	// return (v[0] + v[2]) + 2*v[1];
	// 10 Hz
	v[2] = (9.446918438402e-4 * x)
				 + ( -0.9149758348 * v[0])
				 + (  1.9111970674 * v[1]);
			return
				 (v[0] + v[2]) +2* v[1];
}
