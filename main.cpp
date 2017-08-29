#include "StateSpace.hpp"
#include <Eigen>
#include <iostream>

#define STATES   	3 	 //Number of States
#define INPUTS   	1	 //Number of Inputs
#define OUTPUTS		1 	 //Number of Outputs
#define INERTIA 	0.01 //Rotor Interia
#define FRICTION	0.1  //viscous coefficient of friction
#define SMALL_K		0.01 //
#define RESISTANCE 	1	 //armature resistance
#define INDUCTANCE 	0.5  //armature inductance


int main(){



	StateSpace model(STATES,INPUTS,OUTPUTS,0); //3 states, 1 input, 1 output, Full State Feedback
	
	//Set Control/ System Parameters
	Eigen::MatrixXf A(STATES,STATES);
	Eigen::MatrixXf B(STATES,INPUTS);
	Eigen::MatrixXf C(OUTPUTS,STATES);
	Eigen::MatrixXf K(INPUTS,STATES);
	Eigen::MatrixXf r(OUTPUTS,1);




	A<< 0,	1,						0,
		0,	-FRICTION/INERTIA,		SMALL_K/INERTIA,
		0,	-SMALL_K/INDUCTANCE,	-RESISTANCE/INDUCTANCE;

	B<< 0,
		0,
		1/INDUCTANCE;

	C<< 1, 0, 0;

	K<< 1, 5, 4; //Calculated values

	r<< 3;


	model.Initialise(A,B,C,K); //D is 0, not included at this time
	model.setReferenceInputs(r);

	for(int i = 0; i < 5000; i++){
		model.Calculate();
		std::cout<<model.getRealOutput()<<'\n';
	}
		



	return 0;
}

