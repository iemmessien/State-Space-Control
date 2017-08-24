/*
	File: StateSpace.hpp
	Description: A Class for creating state space calculations 
	
	TODO:

*/
#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include <Eigen>



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
	Eigen::MatrixXf estimatorOutput;

public:
	StateSpace(int state, int input, int output, int type);
	void Initialise(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf K);
	void Update(/*Matrix<states> &systemState, float dt*/);
	
	
	

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
	

};
#endif // STATE_SPACE_CONTROL_H



