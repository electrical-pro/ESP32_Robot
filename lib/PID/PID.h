/*
See .cpp file for library explanations

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
*/

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define cP 			(uint8_t) 0x00
#define cPI 		(uint8_t) 0x01
#define cPD 		(uint8_t) 0x02
#define cLP 		(uint8_t) 0x04
#define cPID 		cPI | cPD
#define cP_LP	 	cLP
#define cPI_LP	cPI | cLP
#define cPD_LP	cPD | cLP
#define cPID_LP cPI | cPD | cLP


class PID {
	public:
		PID(void);
		PID(uint8_t controllerType, float dT, float max, float min);

		void setParameters(float nK, float nTi, float nTd, float nN);
		void updateParameters();
		float calculate();
		void resetITerm();
		void resetDTerm();
		void reset();

		float setpoint, input;
		uint8_t intDeadband;
		float minOutput, maxOutput;

		uint8_t controllerType;
		float K, Ti, Td, N, R;

		bool resetInt;

	private:
		void init();
		float outputFilter(float x);

		float _dT;
		float _lastDterm;
		float _sumError;
		float _Pterm, _Iterm, _Dterm;
		float _lastError, _lastInput, _lastSetpoint;
		float _if, _df1, _df2;
		float v[3];



};

#endif
