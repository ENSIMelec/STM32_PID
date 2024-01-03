#ifndef DISCRETEPID
#define DISCRETEPID

/** Class to compute PID control filter in discrete form **/
class DiscretePID { 
	private: 
		float q0,  ///< Q0 coefficient
		      q1,  ///< Q1 coefficient
		      q2,  ///< Q2 coefficient
		      em1, ///< Error at previous sample
		      em2, ///< Error two samples ago
		      y;   ///< Filter's output
	public:
		/** Initialize PID filter
		  * @param [in] dt Sampling period in seconds
		  * @param [in] Kp Proportionnal coefficient
		  * @param [in] Ki Integral coefficient
		  * @param [in] Kd Derivative coefficient
		  */
		DiscretePID(float dt, float Kp, float Ki, float Kd); 

		/** Process sampled value
		  * @note Should be called only once per sample
		  * @param [in] error Sampled error
		  * @return Filters's output
		  */
		float process(float error); 
}; 

DiscretePID::DiscretePID(float dt, float Kp, float Ki, float Kd) : 
	q0(Kp*(Ki*dt/2+Kd/dt+1)), 
	q1(Kp*(Ki*dt/2-Kd/dt*2-1)), 
	q2(Kp*Kd/dt), 
	em1(0), 
	em2(0), 
	y(0) { 
} 

float DiscretePID::process(float e) { 
	y += q0 * e + q1 * em1 + q2 * em2; 
	em2 = em1; 
	em1 = e; 
	return y; 
}

#endif