#ifndef CONTROLLER_H
#define CONTROLLER_H
/* 
These are the controllers proposed in : 
A. Pourkand and J. J. Abbott, "Magnetic Actuation With Stationary Electromagnets Considering Power and Temperature Constraints," 
in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6964-6971, Oct. 2020, doi: 10.1109/LRA.2020.3025512.


*/

#include "type.hpp"


class Controller{
	private:
		vec_current* Amp_I_lim;
		current* Pow_I_lim;
		
	public:
		void Init(vec_current* Amp_I_lim, current* Pow_I_lim);
		void PINV(vec_wrench* output, Mat_actuation* A, vec_current* I_output);
		void WPINV(vec_wrench* output, Mat_actuation* A, vec_temp_C* coils_temp, vec_current* I_output);
		void RWPINV(vec_wrench* output, Mat_actuation* A, vec_temp_C* coils_temp, vec_current* I_output);
		void AMPScale();
		void AmplifierScale();
		void PowerSupplyScale();
		void WeightMatGen();
};
#endif // CONTROLLER_H
