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
	


    // State space matrices
    Eigen::VectorXd systemMatrix;
    Eigen::VectorXd inputMatrix;
    Eigen::VectorXd outputMatrix;
    Eigen::VectorXd transmissionMatrix;
    Eigen::VectorXd controlInputs;
    Eigen::VectorXd referenceInputs;

    // Gains
    Eigen::VectorXd controlGain;
    Eigen::VectorXd integralGain;
	
	// Things to precalculate
    Eigen::VectorXd precompensator;
	Eigen::VectorXd estimatorOutput;

public:
	StateSpace(int state, int input, int output, int type);
	void Initialise();
	void Update(/*Matrix<states> &systemState, float dt*/);
	
	
	
	//Get & SetFunctions
	Eigen::VectorXd getControlInputs();
	Eigen::VectorXd getReferenceInputs();
	Eigen::VectorXd getSystemMatrix();
	Eigen::VectorXd getInputMatrix();
	Eigen::VectorXd getOutputMatrix();
	Eigen::VectorXd getTransmissionMatrix();
	Eigen::VectorXd getControlGain();
	Eigen::VectorXd getIntegralGain();
	Eigen::VectorXd getPrecompensator();
	Eigen::VectorXd getEstimatorOutput();

	
	void setControlInputs(Eigen::VectorXd X);
	void setReferenceInputs(Eigen::VectorXd X);
	void setSystemMatrix(Eigen::VectorXd X);
	void setInputMatrix(Eigen::VectorXd X);
	void setOutputMatrix(Eigen::VectorXd X);
	void setTransmissionMatrix(Eigen::VectorXd X);
	void setControlGain(Eigen::VectorXd X);
	void setIntegralGain(Eigen::VectorXd X);
	void setPrecompensator(Eigen::VectorXd X);
	void setEstimatorOutput(Eigen::VectorXd X);
	

};
#endif // STATE_SPACE_CONTROL_H



