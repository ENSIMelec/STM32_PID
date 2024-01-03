#ifndef SIMFIRSTORDER
#define SIMFIRSTORDER

/** Class to simulate first order system **/
class SimFirstOrder { 
	private: 
		float E, ///< Constant, depending on sampling rate and time constant
		      K, ///< Static gain
		      y; ///< Last output
	public:
		/** Initialize first order simulator
		  * @param [in] period Sampling period
		  * @param [in] tau    System time constant
		  * @param [in] k      System static gain
		  */
		SimFirstOrder(float period, float tau, float k = 1);

		/** Compute the system response
		 * @param [in] c Command to apply
		 * @return New value
		 */
		float process(float c);
}; 

SimFirstOrder::SimFirstOrder(float period, float tau, float k) : 
	E(exp(-period / tau)), 
	K(k), 
	y(0) { 
} 

float SimFirstOrder::process(float c) {
	y = K * c + (y - K * c) * E;
	return y;
}

#endif