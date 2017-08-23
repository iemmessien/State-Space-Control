#include "StateSpace.hpp"

// 0 for FullState   
//1 for Autonomous

    StateSpace::StateSpace(int state, int input, int output, int enumtype)
    {
		inputs = input;
		outputs = output;
		states = state;
		type = enumtype;
		
        // Zero out all the matrices
        systemMatrix.resize(states,states);  //A
	
		inputMatrix.resize(states,inputs);  //B
		outputMatrix.resize(outputs, states); //C
		transmissionMatrix.resize(outputs, inputs);
        //controlInputs.resize();
		//referenceInputs.resize();
        
        controlGain.resize(inputs,states); //K
		integralGain.resize(outputs,outputs);
		precompensator.resize(inputs,outputs);
		estimatorOutput.resize(states,states);
		
	
		
    }
	
	 void StateSpace::Initialise()
    {
		/*
		// Aggregate all the state space matrices into one big matrix
		Matrix<states + outputs, states + inputs> sys;
		sys.Submatrix(Slice<0,states>(),Slice<0,states>()) = systemMatrix;
		sys.Submatrix(Slice<0,states>(),Slice<states,states+inputs>()) = inputMatrix;
		sys.Submatrix(Slice<states,states+outputs>(),Slice<0,states>()) = outputMatrix;
		sys.Submatrix(Slice<states,states+outputs>(),Slice<states,states+inputs>()) = directTransmissionMatrix;
		
		// Find an inverse for the aggregated matrix
		Matrix<states + inputs, states + outputs> sysInv = sys.Transpose() * (sys * sys.Transpose()).Inverse();

		// Split it up and multiply it with K to find NBar
		precompensator = controlGain * sysInv.Submatrix(Slice<0,states>(),Slice<states,states+outputs>()) + sysInv.Submatrix(Slice<states,states+inputs>(),Slice<states,states+outputs>());

		// We can save a bit of time by precalculating the (F - L * H) term in the state update equation
		systemEstimatorOutput = systemMatrix - estimatorGain * outputMatrix;
	*/	
	}
	
	void StateSpace::Update(/*Matrix<states> &systemState, float dt*/)
    {
    	/*
		switch (this->type){
		case 1:
        // Recalculate the control input
        controlInput =  precompensator * referenceInput - controlGain * systemState;

        // Windup the reference input
        referenceInput += integralGain * (referenceInput - outputMatrix * systemState) * dt;
		break;
		
		case 2:
		 // autonomous estimator Fig 7.49(b):  x_dot = (F - L * H) * x_bar + G * u + L * y
        stateEstimate += (systemEstimatorOutput * stateEstimate + inputMatrix * controlInput + estimatorGain * systemOutput) * dt;

        // Recalculate the control input
        controlInput =  precompensator * referenceInput - controlGain * stateEstimate;

        // Windup the reference input
        referenceInput += integralGain * (referenceInput - outputMatrix * stateEstimate) * dt;
		break;		
		}

		*/
	}
	
	
	//Get & Set Functions
	Eigen::VectorXd StateSpace::getControlInputs(){
		return controlInputs;
	}
	Eigen::VectorXd StateSpace::getReferenceInputs(){
		return referenceInputs;
	}
	Eigen::VectorXd StateSpace::getSystemMatrix(){
		return systemMatrix;
	}
	Eigen::VectorXd StateSpace::getInputMatrix(){
		return inputMatrix;
	}
	Eigen::VectorXd StateSpace::getOutputMatrix(){
		return outputMatrix;
	}
	Eigen::VectorXd StateSpace::getTransmissionMatrix(){
		return transmissionMatrix;
	}
	Eigen::VectorXd StateSpace::getControlGain(){
		return controlGain;
	}
	Eigen::VectorXd StateSpace::getIntegralGain(){
		return integralGain;
	}
	Eigen::VectorXd StateSpace::getPrecompensator(){
		return precompensator;
	}
	Eigen::VectorXd StateSpace::getEstimatorOutput(){
		return estimatorOutput;
	}

	
	void StateSpace::setControlInputs(Eigen::VectorXd X){
		controlInputs = X;
	}
	
	void StateSpace::setReferenceInputs(Eigen::VectorXd X){
		referenceInputs = X;
	}
	void StateSpace::setSystemMatrix(Eigen::VectorXd X){
		systemMatrix = X;
	}
	void StateSpace::setInputMatrix(Eigen::VectorXd X){
		inputMatrix = X;
	}
	void StateSpace::setOutputMatrix(Eigen::VectorXd X){
		outputMatrix = X;
	}
	void StateSpace::setTransmissionMatrix(Eigen::VectorXd X){
		transmissionMatrix = X;
	}
	void StateSpace::setControlGain(Eigen::VectorXd X){
		controlGain = X;
	}
	void StateSpace::setIntegralGain(Eigen::VectorXd X){
		integralGain = X;
	}
	void StateSpace::setPrecompensator(Eigen::VectorXd X){
		precompensator = X;
	}
	void StateSpace::setEstimatorOutput(Eigen::VectorXd X){
		estimatorOutput = X;
	}



