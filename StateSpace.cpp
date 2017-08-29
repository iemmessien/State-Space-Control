#include "StateSpace.hpp"

// 0 for FullState   
//1 for Autonomous (to be implamented)

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
		transmissionMatrix.resize(outputs, inputs); //D (will always be 0; included for potential future use)
        controlInputs.resize(inputs, 1); //u
		referenceInputs.resize(outputs, 1); //r
        
        controlGain.resize(inputs,states); //K
		integralGain.resize(outputs,outputs); 
		compensator.resize(states,inputs); //N_x
		precompensator.resize(inputs,outputs); //N_u
		estimatorOutput.resize(states,states); //L

		actual.resize(states,outputs); //actual state of the system
		realOutput.resize(1,outputs); //output of the system

		//Sets all matrices to 0 until the system is initialised
		systemMatrix.fill(0);  
		inputMatrix.fill(0); 
		outputMatrix.fill(0); 
		transmissionMatrix.fill(0); 
        controlInputs.fill(0);
		referenceInputs.fill(0);
        controlGain.fill(0); 
		integralGain.fill(0); 
		precompensator.fill(0); 
		compensator.fill(0);
		estimatorOutput.fill(0);
		actual.fill(0);
		realOutput.fill(0);

		
    }
	
	 void StateSpace::Initialise(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf K)
    {
    	systemMatrix = A;
    	inputMatrix  = B;
    	outputMatrix = C;
    	controlGain  = K;

    	//Create entire system matrix to Calculate 
    	Eigen::MatrixXf sys(states+1, states+1);
    	Eigen::MatrixXf sysInv(states+1, states+1);
    	Eigen::MatrixXf N(states+1, inputs);
    	Eigen::MatrixXf BigZero(states+1, inputs); //for a 1 input system, all values are 0 except last element, unsure of the math if multiple inputs


    	sys.topLeftCorner(states,states) 	  = systemMatrix;
    	sys.topRightCorner(states,inputs) 	  = inputMatrix;
    	sys.bottomLeftCorner(outputs,states)  = outputMatrix;
    	sys.bottomRightCorner(outputs, inputs)= transmissionMatrix;

    	
    	sysInv = sys.inverse();

  

    	BigZero.fill(0); 
    	BigZero.row(states).tail(1) << 1;  //*****TEMPORARY SOLUTION ONLY WORKS WITH 1 INPUT AND OUTPUT*****

    	N = sysInv * BigZero;

    	

    	precompensator = N.topLeftCorner(outputs,inputs);
    	compensator = N.bottomLeftCorner(states,inputs);
	
	}
	
	Eigen::MatrixXf StateSpace::Calculate()
    {
    	actual += (systemMatrix * actual + inputMatrix * controlInputs)*0.1;
	
		realOutput = outputMatrix * actual;

		
        controlInputs =  precompensator * referenceInputs - controlGain * (actual-compensator);

     
        referenceInputs += integralGain * (referenceInputs - outputMatrix * actual) * 0.1;

		return realOutput;
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
	Eigen::MatrixXf StateSpace::getActual(){
		return actual;
	}
	Eigen::MatrixXf StateSpace::getRealOutput(){
		return realOutput;
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
	void StateSpace::setActual(Eigen::MatrixXf X){
		actual = X;
	}
	void StateSpace::setRealOutput(Eigen::MatrixXf X){
		realOutput = X;
	}



