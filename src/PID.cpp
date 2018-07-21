#include "PID.h"
#include <limits>
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool doesTwiddle) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = numeric_limits<double>::max();
	i_error = 0.0;
	d_error = 0.0;

    this->doesTwiddle = doesTwiddle;
    total_error = 0.0;
    best_error = std::numeric_limits<double>::max();
    tolerance = 0.2;
    moveCount = 1;
    n_settle_moves = 100;
    //n_eval_moves = 2000;
    n_eval_moves = 200;	// 2 * n_settle_moves
}

void PID::UpdateError(double cte) {
	if (p_error == numeric_limits<double>::max())
		p_error = cte;
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;


    // twiddle variables
    p = {this->Kp, this->Ki, this->Kd};
    dp = {0.1*this->Kp, 0.1*this->Ki, 0.1*this->Kd};

    // update total error only if we're past number of settle moves
    //if (doesTwiddle && moveCount % (n_settle_moves + n_eval_moves) > n_settle_moves){
    if (doesTwiddle && moveCount % n_eval_moves > n_settle_moves){
        total_error += pow(cte, 2);
    }

    // twiddle loop here
    if (doesTwiddle && moveCount % n_eval_moves == 0) {
    	while (dp[0]+dp[1]+dp[2] > tolerance) {

        	for (int i = 0; i < p.size(); i++) {
            	p[i] += dp[i];

            	if (total_error < best_error) {
                	best_error = total_error;
                	dp[i] *= 1.1;
            	} else {
                	p[i] -= 2 * dp[i];

                	if (total_error < best_error) {
                    	best_error = total_error;
                    	dp[i] *= 1.1;
                	} else {
                    	p[i] += dp[i];
                    	dp[i] *= 0.9;
                	}
            	}
        	}
    	}
        	
        total_error = 0;
	   		
	   	this->Kp = p[0];
   		this->Ki = p[1];
   		this->Kd = p[2];
 
        cout << "" << endl;
        cout << "=================================  New twiddled parameters - P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
 	}
   	
    moveCount++;
}

double PID::TotalError() {
	return Kp*p_error + Ki*i_error + Kd*d_error;
}

