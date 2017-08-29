/*
	File: StateSpace.hpp
	Description: A Class for creating state space calculations 
	
	TODO: Finish full state
		  Write autonomous

*/
#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include <Eigen>
#include <iostream>


class StateSpace{
private:
	int states;
	int inputs;
	int outputs;
	int type;

    
    Eigen::MatrixXf systemMatrix;
    Eigen::MatrixXf inputMatrix;
    Eigen::MatrixXf outputMatrix;
    Eigen::MatrixXf transmissionMatrix;
    Eigen::MatrixXf controlInputs;
    Eigen::MatrixXf referenceInputs;
    Eigen::MatrixXf controlGain;
    Eigen::MatrixXf integralGain;
    Eigen::MatrixXf precompensator;
    Eigen::MatrixXf compensator;
	Eigen::MatrixXf estimatorOutput;

	//Set Actual State & Output
	Eigen::MatrixXf actual;
	Eigen::MatrixXf	realOutput;

public:
	StateSpace(int state, int input, int output, int type);
	void Initialise(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf K);  // Sets Values of matrices to values defined in the main
	Eigen::MatrixXf Calculate();	//Determines the output of the system and updates new values
	
	
	

	//Get Functions
	Eigen::MatrixXf getControlInputs();
	Eigen::MatrixXf getReferenceInputs();
	Eigen::MatrixXf getSystemMatrix();
	Eigen::MatrixXf getInputMatrix();
	Eigen::MatrixXf getOutputMatrix();
	Eigen::MatrixXf getTransmissionMatrix();
	Eigen::MatrixXf getControlGain();
	Eigen::MatrixXf getIntegralGain();
	Eigen::MatrixXf getPrecompensator();
	Eigen::MatrixXf getEstimatorOutput();
	Eigen::MatrixXf getActual();
	Eigen::MatrixXf getRealOutput();

	//Set Functions
	void setControlInputs(Eigen::MatrixXf X);
	void setReferenceInputs(Eigen::MatrixXf X);
	void setSystemMatrix(Eigen::MatrixXf X);
	void setInputMatrix(Eigen::MatrixXf X);
	void setOutputMatrix(Eigen::MatrixXf X);
	void setTransmissionMatrix(Eigen::MatrixXf X);
	void setControlGain(Eigen::MatrixXf X);
	void setIntegralGain(Eigen::MatrixXf X);
	void setPrecompensator(Eigen::MatrixXf X);
	void setEstimatorOutput(Eigen::MatrixXf X);
	void setActual(Eigen::MatrixXf X);
	void setRealOutput(Eigen::MatrixXf X);

};
#endif // STATE_SPACE_CONTROL_H



