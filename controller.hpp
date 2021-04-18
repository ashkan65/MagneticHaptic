#ifndef CONTROLLER_H
#define CONTROLLER_H
/* 
These are the controllers proposed in : 
A. Pourkand and J. J. Abbott, "Magnetic Actuation With Stationary Electromagnets Considering Power and Temperature Constraints," 
in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6964-6971, Oct. 2020, doi: 10.1109/LRA.2020.3025512.

By: Ashkan Pourkand
Apr-2021
*/

#include "type.hpp"


class Controller{

	private:
		bool ext_cost = false;																	// Flag to use external cost function for temperature
		// double (*) ext_T_cost_function;														// Calls the external temperature cost function --> TODO!
		temperature critical_temp = 150;														// This is hard coded for now!
		uint number_coils;																		// Total numbers of coils (count how many amplifier you have)
		uint DOF;																				// Degrees of actuation (norma magnets are only 5)
		current powersupply_I_lim;																// The current saturaiton limit in power supply		
		vec_current* amp_I_lim_H;																// Upper current limit of amplifier 
		vec_current* amp_I_lim_L;																// Lower current limit of amplifier 		
		// Eigen::MatrixXd* w;																	// The weight matrix in that keeps the cost of the AVAILABLE coils
		std::vector<uint>::iterator it; 														// Use this to find a value in a std::vector			
		double L1(vec_current* I);																										// DONE!
		void ScaleAmplifier(vec_current* I_MaxHigh, vec_current* I_MaxLow, vec_current* Is, double* Gamma, std::vector<uint>& arg);		//Done!
		void ScalePowerSupply(vec_current* I_prev, vec_current* I_new, current* I_pow, double* Gamma);									//Done!		
		void pop(Eigen::VectorXi* v, uint i);																							// pop(&v, i)-> pops the ith elements in v		DONE!
		void raw_WPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output);			// This isthe raw wpinv. It does not consider the saturation limits. DO NOT USE IT!
		void RPINV(vec_wrench& wrench, Mat_actuation& A, std::vector<uint>& available_coils, vec_current& I_old, uint null_space, vec_current& I_output);	// This is being used in recersive calls. DO NOT USE IT!
		void RWPINV(vec_wrench& wrench, Mat_actuation& A, std::vector<uint>& _available_coils, vec_current& I_old, uint null_space, vec_temp_C& T, vec_current& I_output);		// This is being used in recersive calls. DO NOT USE IT!
		void GetWeightMat(vec_temp_C& T, Eigen::MatrixXd& w);									// updates the W matrix using in WPINV and RWPINV. 	
		double T_cost(double);																	// Cost function for the temperature
	public:
		Controller(uint number_couls, double  _amp_I_lim, double _powersupply_I_lim, uint DOF);
		~Controller();
		void AddTCostFunction(double (*T_cost_function)(double));								// Gets the pointer to the cost function, you can also use the cost function from the class. 
		void PINV(vec_wrench& wrench, Mat_actuation& A, vec_current& I_output);					// Done! This is the normal pseudo inverse. 
		void RPINV(vec_wrench& wrench, Mat_actuation& A, vec_current& I_output);				// Done! This is the recursive pseudo inverse.
		// void WPINV(vec_wrench& wrench, Mat_actuation& A, std::vector<uint>& _available_coils, vec_current& I_old, uint null_space, vec_temp_C& T, vec_current& I_output);
		void WPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output);
		void RWPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output);
};
#endif // CONTROLLER_H
