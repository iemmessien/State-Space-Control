#include "StateSpace.hpp"
#include <Eigen>
#include <iostream>

#define STATES   	3
#define INPUTS   	1
#define OUTPUTS		1 
#define INERTIA 	0.01 //Rotor Interia
#define FRICTION	0.1  //viscous coefficient of friction
#define SMALL_K		0.01 //
#define RESISTANCE 	1	 //armature resistance
#define INDUCTANCE 	0.5  //armature inductance


int main(){



	StateSpace s(STATES,INPUTS,OUTPUTS,0); //3 states, 1 input, 1 output, Full State Feedback
	Eigen::MatrixXf A(STATES,STATES);
	Eigen::MatrixXf B(STATES,INPUTS);
	Eigen::MatrixXf C(OUTPUTS,STATES);
	Eigen::MatrixXf K(INPUTS,STATES);

	A<< 0,	1,						0,
		0,	-FRICTION/INERTIA,		SMALL_K/INERTIA,
		0,	-SMALL_K/INDUCTANCE,	-RESISTANCE/INDUCTANCE;

	B<< 0,
		0,
		1/INDUCTANCE;

	C<< 1, 0, 0;

	K<< 2, 32.9, 4; //Calculated values

	s.Initialise(A,B,C,K);


	return 0;
}

