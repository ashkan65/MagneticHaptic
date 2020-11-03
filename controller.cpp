#include "controller.hpp"


void Controller::Init(vec_current* Amp_I_lim, current* Pow_I_lim){

};
void Controller::PINV(vec_wrench* output, Mat_actuation* A, vec_current* I_output){

};
void Controller::WPINV(vec_wrench* output, Mat_actuation* A, vec_temp_C* coils_temp, vec_current* I_output){

};
void Controller::RWPINV(vec_wrench* output, Mat_actuation* A, vec_temp_C* coils_temp, vec_current* I_output){

};
void Controller::AMPScale(){

};
void Controller::AmplifierScale(){

};
void Controller::PowerSupplyScale(){

};
void Controller::WeightMatGen(){

};