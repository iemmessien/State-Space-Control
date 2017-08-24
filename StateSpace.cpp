#include "StateSpace.hpp"

// 0 for FullState   
//1 for Autonomous

    StateSpace::StateSpace(int state, int input, int output, int enumtype)
    {
		inputs = input;
		outputs = output;
		states = state;
		type = enumtype;
		
        // Establish proper size for matrices: Comments represent common notation in block diagrams
        systemMatrix.resize(states,states);  //A
		inputMatrix.resize(states,inputs);  //B
		outputMatrix.resize(outputs, states); //C
		transmissionMatrix.resize(outputs, inputs); //D (will always be 0)
        //controlInputs.resize();
		//referenceInputs.resize();
        
        controlGain.resize(inputs,states); //K
		integralGain.resize(outputs,outputs);
		precompensator.resize(inputs,outputs); //N_bar
		estimatorOutput.resize(states,states); //L
		
	
		
    }
	
	 void StateSpace::Initialise(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf K)
    {
    	systemMatrix = A;
    	inputMatrix  = B;
    	outputMatrix = C;
    	controlGain  = K;

    	transmissionMatrix.fill(0);
	
	}
	
	void StateSpace::Update(/*Matrix<states> &systemState, float dt*/)
    {

	}
	
	
	//Get & Set Functions
	Eigen::MatrixXf StateSpace::getControlInputs(){
		return controlInputs;
	}
	Eigen::MatrixXf StateSpace::getReferenceInputs(){
		return referenceInputs;
	}
	Eigen::MatrixXf StateSpace::getSystemMatrix(){
		return systemMatrix;
	}
	Eigen::MatrixXf StateSpace::getInputMatrix(){
		return inputMatrix;
	}
	Eigen::MatrixXf StateSpace::getOutputMatrix(){
		return outputMatrix;
	}
	Eigen::MatrixXf StateSpace::getTransmissionMatrix(){
		return transmissionMatrix;
	}
	Eigen::MatrixXf StateSpace::getControlGain(){
		return controlGain;
	}
	Eigen::MatrixXf StateSpace::getIntegralGain(){
		return integralGain;
	}
	Eigen::MatrixXf StateSpace::getPrecompensator(){
		return precompensator;
	}
	Eigen::MatrixXf StateSpace::getEstimatorOutput(){
		return estimatorOutput;
	}

	
	void StateSpace::setControlInputs(Eigen::MatrixXf X){
		controlInputs = X;
	}
	
	void StateSpace::setReferenceInputs(Eigen::MatrixXf X){
		referenceInputs = X;
	}
	void StateSpace::setSystemMatrix(Eigen::MatrixXf X){
		systemMatrix = X;
	}
	void StateSpace::setInputMatrix(Eigen::MatrixXf X){
		inputMatrix = X;
	}
	void StateSpace::setOutputMatrix(Eigen::MatrixXf X){
		outputMatrix = X;
	}
	void StateSpace::setTransmissionMatrix(Eigen::MatrixXf X){
		transmissionMatrix = X;
	}
	void StateSpace::setControlGain(Eigen::MatrixXf X){
		controlGain = X;
	}
	void StateSpace::setIntegralGain(Eigen::MatrixXf X){
		integralGain = X;
	}
	void StateSpace::setPrecompensator(Eigen::MatrixXf X){
		precompensator = X;
	}
	void StateSpace::setEstimatorOutput(Eigen::MatrixXf X){
		estimatorOutput = X;
	}



