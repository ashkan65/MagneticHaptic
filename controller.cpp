#include "controller.hpp"


Controller::Controller(uint _number_coils, double  _amp_I_lim, double _powersupply_I_lim, uint _DOF){
    DOF = _DOF;                                             // saving the degrees of freedom of the output
    number_coils = _number_coils;                           // Number of coils in the system
    amp_I_lim_H = new Eigen::VectorXd(_number_coils);
    amp_I_lim_H->Zero(_number_coils);
    amp_I_lim_L = new Eigen::VectorXd(_number_coils); 
    amp_I_lim_L->Zero(_number_coils);
    // w = new Eigen::MatrixXd(_number_coils,_number_coils);
    // For some reason the syntax fto set w to zero doesn't work! I moved the zeroing to the for loop
    powersupply_I_lim = _powersupply_I_lim;
    amp_I_lim_H->fill(_amp_I_lim);
    amp_I_lim_L->fill(-1.0 * _amp_I_lim);
    for (int i = 0 ; i<_number_coils ; i++){
        // available_coils.push_back ((uint)i); 
        (*amp_I_lim_H)(i) = _amp_I_lim;
        (*amp_I_lim_L)(i) = -1.0 * _amp_I_lim;   
        // for (int j = 0 ; j<_number_coils ; j++){(*w)(i,j)= 0.0;};
    }
};


Controller::~Controller(){
    delete amp_I_lim_H;
    delete amp_I_lim_L;
    // delete w;
};


void Controller::PINV(vec_wrench& wrench, Mat_actuation& A, vec_current& I_output){
    // calculates the needed current for the desiered wrench
    vec_current I_zero = Eigen::VectorXd::Zero(number_coils);    		
    I_output = A.completeOrthogonalDecomposition().solve(wrench);     // This is the same as pinv function except it has less error
    // Check to see if any amplifier is saturated;
    double gamma_amp, gamma_pow;
    std::vector<uint> arg;
    ScaleAmplifier(amp_I_lim_H, amp_I_lim_L, &I_output, &gamma_amp, arg);
    ScalePowerSupply(&I_zero, &I_output, &powersupply_I_lim, &gamma_pow);
    double Gamma = std::min((double)std::min(gamma_amp,gamma_pow),1.0);
    // if (gamma_pow <=1){
    //     std::cout<<"Power saturation: "<<gamma_pow<<std::endl;
    // }
    // if (gamma_amp !=1){
    //     std::cout<<"Amp saturation: "<<gamma_amp<<std::endl;
    // }
    if ((gamma_amp <1) || (gamma_pow <=1)){
        I_output = Gamma * I_output; 
    }
};


void Controller::ScaleAmplifier(vec_current* I_MaxHigh, vec_current* I_MaxLow, vec_current* Is, double* Gamma, std::vector<uint>& arg){
    *Gamma = 1.0;
    for (uint i=0; i<Is->size();i++){
        if (((*Is)[i]>0) && ((*I_MaxHigh)[i]/(*Is)[i]<*Gamma)){
            if((*I_MaxHigh)[i]!=0){
                *Gamma = (((*I_MaxHigh)[i])/((*Is)[i]));
                arg.push_back(i);
            }
        }
        if (((*Is)[i]<0) && ((*I_MaxLow)[i]/(*Is)[i]<*Gamma)) {
            if((*I_MaxLow)[i]!=0){
                *Gamma = ((*I_MaxLow)[i])/((*Is)[i]);
                arg.push_back(i);
            }
        }
    }
};


void Controller::ScalePowerSupply(vec_current* I_prev, vec_current* I_new, current* I_pow, double* Gamma){
    double error = 100.0;
    // vec_current * II = (*I_prev + *I_new);
    vec_current I_m, I_M;
    I_m = *I_prev + *I_new;
    double lower = 1/(L1(&I_m)/(*I_pow));
    double upper = 1/(L1(I_new)/((double)(*I_pow) - L1(I_prev)));
    double mid;
    while (abs(error)>0.1){
        mid = 0.5*(lower + upper);
        I_M = (*I_prev) + mid * (*I_new);
        error = *I_pow - L1(&I_M);
        if (error < 0){
            lower = mid;
        }    
        else{
            upper = mid;
        }
        if (abs(lower - upper) <0.001){
            error = 0;
        }
    }
    *Gamma = mid;
// end
};


void Controller::GetWeightMat(vec_temp_C& T, Eigen::MatrixXd& w){
    int n_coils = T.size();
    // w.resize(n_coils,n_coils);
    // w.Identity(n_coils,n_coils);
    // (*w).Zero();
    for (int i = 0; i<n_coils ; i++){
        for (int j = 0; j<n_coils ; j++){
            if (i==j){
                w(i,j) = T_cost(T(i));
            }
            else{
                w(i,j) = 0.0;
            }
        }
    }
};


double Controller::L1(vec_current* I){
    return I->cwiseAbs().sum();
};


void Controller::AddTCostFunction(double (*T_cost_function)(double)){
    // Gets the pointer to the cost function, you can also use the cost function from the class. 
    ext_cost = true;
    // ext_T_cost_function = T_cost_function;
};							


double Controller::T_cost(temperature T){
    if (T < critical_temp)
        return 1;
    else
        return  pow(exp((T-critical_temp)),4);
};


void Controller::pop(Eigen::VectorXi* v, uint i){
    uint l = v->size();
    Eigen::VectorXi _v = *v;
    v->conservativeResize(l-1,1);
    v->tail(l-i-1) = _v.tail(l-i-1);
}


void Controller::RPINV(vec_wrench& wrench, Mat_actuation& A, vec_current& I_output){
    std::vector<uint> available_coils;	// Which amplifiers are not saturated	
    vec_current I_old(number_coils);
	// Which amplifiers are not saturated	
    for (uint i = 0; i<number_coils;i++){
        available_coils.push_back(i);
        I_old(i) = 0.0; 
    }
    RPINV(wrench, A, available_coils, I_old, (uint)(number_coils-DOF), I_output);
};


void Controller::RPINV(vec_wrench& wrench, Mat_actuation& A, std::vector<uint>& _available_coils, vec_current& I_old, uint null_space,vec_current& I_output){
    Mat_actuation actuation_temp(DOF, _available_coils.size()); 
    for (int j =0 ; j<_available_coils.size() ; j++){
        actuation_temp.col(j) = A.col(_available_coils[j]);                             //      act = A(:,Avail_coils);
    }
    Eigen::VectorXd I(number_coils);
    for (int i =0 ; i<number_coils ; i++){
        I(i) = 0.0;                                                                     //      I(Avail_coils(i)) = I_temp(i);
    }
    vec_current I_temp (_available_coils.size());
    I_temp = actuation_temp.completeOrthogonalDecomposition().solve(wrench);            //      I_temp  = pinv(act) * Wrench;
    for (int i =0 ; i<_available_coils.size() ; i++){
        I(_available_coils[i]) = I_temp(i);                                             //      I(Avail_coils(i)) = I_temp(i);
    }
    double gamma_amp, gamma_pow;
    std::vector<uint> arg;
    vec_current new_high(number_coils);
    new_high = (*amp_I_lim_H)-I_old;
    vec_current new_low(number_coils);
    new_low = (*amp_I_lim_L)-I_old;    
    ScaleAmplifier(&new_high, &new_low, &I, &gamma_amp, arg);                           //     [Gamma_amp,arg] = ScaleAmp(I_MaxHigh - I_old, I_MaxLow - I_old, I);
    if (gamma_amp <1){
        // std::cout<<"Amp saturation: "<<gamma_amp<<std::endl;
        for (int i = 0; i< arg.size(); i++){
            _available_coils.erase(std::remove(_available_coils.begin(), _available_coils.end(), arg[i]), _available_coils.end()); //Avail_coils(find(Avail_coils == arg(i))) = [];
        }
    }

    I = gamma_amp * I;                                                                  //     I = Gamma_amp * I;

    ScalePowerSupply(&I_old, &I, &powersupply_I_lim, &gamma_pow);                       //     Gamma_pow = ScalePow(I_old, I, I_pow)   -> This does the bysection!
    // if (gamma_pow <=1){
    //     std::cout<<"Power saturation: "<<gamma_pow<<std::endl;
    // }
    if ((gamma_amp ==1) || (gamma_pow <=1) || (null_space == 0))
    {
        if (gamma_pow <=1){
            I_output = gamma_pow * I;                                                   //     I = Gamma_pow * I;
            return;
        }
        else{
            I_output =  I;
            return;
        }
    }
    else{
        vec_wrench wrench_temp(DOF);
        wrench_temp =  wrench - (A * I);
        I_old = I_old + I;
        null_space-=1;
        vec_current I_r(number_coils);
        // I_r.Zero(number_coils);
        for (int i =0 ; i<number_coils ; i++){
            I_r(i) = 0.0;                                                               //      I(Avail_coils(i)) = I_temp(i);
        }
        RPINV(wrench_temp, A, _available_coils, I_old, null_space, I_r);
        I_output = I + I_r;
    }
};	


void Controller::raw_WPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output){
    Eigen::MatrixXd w(T.size(),T.size());
    w.Identity(T.size(),T.size());
    GetWeightMat(T, w);
    w = w.completeOrthogonalDecomposition().pseudoInverse();                //!!!!! W is inverted here!!!!!!!!!!!!!!!!!!    
    I_output = (w*A.transpose()* (A*w*A.transpose()).completeOrthogonalDecomposition().pseudoInverse())*wrench ;                     // I = (pinv(W)*act'*pinv(act*pinv(W)*act'))*des;
};


void Controller::WPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output){
    // Use this instead of the raw funcion
    // calculates the needed current for the desiered wrench
    vec_current I_zero = Eigen::VectorXd::Zero(number_coils);		
    raw_WPINV( wrench, A, T, I_output);     // This is the same as pinv function except it has less error
    // Check to see if any amplifier is saturated;
    double gamma_amp, gamma_pow;
    std::vector<uint> arg;
    ScaleAmplifier(amp_I_lim_H, amp_I_lim_L, &I_output, &gamma_amp, arg);
    ScalePowerSupply(&I_zero, &I_output, &powersupply_I_lim, &gamma_pow);
    double Gamma = std::min((double)std::min(gamma_amp,gamma_pow),1.0);
    // if (gamma_pow <=1){
    //     std::cout<<"Power saturation: "<<gamma_pow<<std::endl;
    // }
    // if (gamma_amp !=1){
    //     std::cout<<"Amp saturation: "<<gamma_amp<<std::endl;
    // }
    if ((gamma_amp <1) || (gamma_pow <=1)){
        I_output = Gamma * I_output; 
    }
};


void Controller::RWPINV(vec_wrench& wrench, Mat_actuation& A, vec_temp_C& T, vec_current& I_output){
    std::vector<uint> available_coils;	// Which amplifiers are not saturated	
    vec_current I_old(number_coils);
	// Which amplifiers are not saturated	
    for (uint i = 0; i<number_coils;i++){
        available_coils.push_back(i);
        I_old(i) = 0.0; 
    }
    RWPINV(wrench, A, available_coils, I_old, (uint)(number_coils-DOF), T, I_output);
};


void Controller::RWPINV(vec_wrench& wrench, Mat_actuation& A, std::vector<uint>& _available_coils, vec_current& I_old, uint null_space, vec_temp_C& T, vec_current& I_output){
    Mat_actuation actuation_temp(DOF, _available_coils.size()); 
    vec_temp_C T_temp(_available_coils.size());
    for (int j =0 ; j<_available_coils.size() ; j++){
        actuation_temp.col(j) = A.col(_available_coils[j]);                             //      act = A(:,Avail_coils);
        T_temp(j) = T(_available_coils[j]); 
    }
    Eigen::VectorXd I(number_coils);
    for (int i =0 ; i<number_coils ; i++){
        I(i) = 0.0;                                                                     //      I(Avail_coils(i)) = I_temp(i);
    }
    vec_current I_temp (_available_coils.size());
    raw_WPINV(wrench, actuation_temp, T_temp, I_temp);
    for (int i =0 ; i<_available_coils.size() ; i++){
        I(_available_coils[i]) = I_temp(i);                                             //      I(Avail_coils(i)) = I_temp(i);
    }
    double gamma_amp, gamma_pow;
    std::vector<uint> arg;
    vec_current new_high(number_coils);
    new_high = (*amp_I_lim_H)-I_old;
    vec_current new_low(number_coils);
    new_low = (*amp_I_lim_L)-I_old;    
    ScaleAmplifier(&new_high, &new_low, &I, &gamma_amp, arg);                           //     [Gamma_amp,arg] = ScaleAmp(I_MaxHigh - I_old, I_MaxLow - I_old, I);
    if (gamma_amp <1){
        // std::cout<<"Amp saturation: "<<gamma_amp<<std::endl;
        for (int i = 0; i< arg.size(); i++){
            _available_coils.erase(std::remove(_available_coils.begin(), _available_coils.end(), arg[i]), _available_coils.end()); //Avail_coils(find(Avail_coils == arg(i))) = [];
        }
    }
    I = gamma_amp * I;                                                                  //     I = Gamma_amp * I;
    ScalePowerSupply(&I_old, &I, &powersupply_I_lim, &gamma_pow);                       //     Gamma_pow = ScalePow(I_old, I, I_pow)   -> This does the bysection!
    // if (gamma_pow <=1){
    //     std::cout<<"Power saturation: "<<gamma_pow<<std::endl;
    // }
    if ((gamma_amp ==1) || (gamma_pow <=1) || (null_space == 0))
    {
        if (gamma_pow <=1){
            I_output = gamma_pow * I;                                                   //     I = Gamma_pow * I;
            return;
        }
        else{
            I_output =  I;
            return;
        }
    }
    else{
        vec_wrench wrench_temp(DOF);
        wrench_temp =  wrench - (A * I);
        I_old = I_old + I;
        null_space-=1;
        vec_current I_r(number_coils);
        // I_r.Zero(number_coils);
        for (int i =0 ; i<number_coils ; i++){
            I_r(i) = 0.0;                                                               //      I(Avail_coils(i)) = I_temp(i);
        }
        RWPINV(wrench_temp, A, _available_coils, I_old, null_space, T, I_r);
        I_output = I + I_r;
    }
};	
