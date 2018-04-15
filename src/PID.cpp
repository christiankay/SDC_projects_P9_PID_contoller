#include "PID.h"
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	PID::Kp_ = Kp;//0.134611
	PID::Ki_ = Ki;//0.000270736
	PID::Kd_ = Kd;//3.05349


	PID::p_error_ = 1;
	PID::i_error_ = 1;
	PID::d_error_ = 1;

	PID::prev_cte = 0.0;
	PID::d_cte;
	PID::i_cte;
	PID::is_initialized_ =  false;

	/*initialize twiggle vector p and dp*/
	PID::found_parameter_ = false;
	PID::count_ = 0;


	

}

double PID::UpdateControl(double cte) {
	/*initialize p_error_ with a first cte*/
	if (is_initialized_ == false) {
		prev_cte = cte;
		is_initialized_ = true;
	}
	/*calculate the errors and set ctes for the next round*/
	else {
		d_cte = cte - prev_cte;
		i_cte += cte;
		prev_cte = cte;

		
	}

	return -Kp_ * cte - Kd_ * d_cte; - Ki_ * i_cte;
}

double PID::TotalError() {
	return fabs(p_error_) + fabs(i_error_) + fabs(d_error_);//-Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;

}

void PID::SearchParameters(double tol, double cte) {


	double p[] = { Kp_, Kd_, Ki_ };
	double dp[] = { p_error_, d_error_ , i_error_ };
	cout << "Current total error: " << PID::TotalError() << " Tolerance: " << tol << endl;
	if (PID::TotalError() > tol) {


		cout << "Current index: " << last_optimize_index << endl;
		switch (current_state) {
		case 0: {
			p[last_optimize_index] += dp[last_optimize_index];
			current_state = 1;
			break;
		}
		case 1: {
			if (fabs(cte) < fabs(best_error)) {
				best_error = cte;
				dp[last_optimize_index] *= 1.05;
				current_state = 3;
			}
			else {
				p[last_optimize_index] -= 2 * dp[last_optimize_index];
				current_state = 2;
			}
			break;
		}
		case 2: {
			if (fabs(cte) < fabs(best_error)) {
				best_error = cte;
				dp[last_optimize_index] *= 1.05;
			}
			else {
				p[last_optimize_index] += dp[last_optimize_index];
				dp[last_optimize_index] *= 0.95;
			}
			current_state = 3;
			break;
		}
		case 3: {
			last_optimize_index = (last_optimize_index + 1) % 3;
			current_state = 0;
			break;
		}
		}
		cout << "Pp " << p[0] << "Pd " << p[1] << "Pi " << p[2] << endl;
		cout << "Searching for parameters - Iteration: " << count_ << endl;
		p_error_ = dp[0];
		d_error_ = dp[1];
		i_error_ = dp[2];
		Kp_ = p[0];
		Kd_ = p[1];
		Ki_ = p[2];
		count_++;
	}

	else
	{
		cout << "Found new parameters!" << '\n';
		cout << "Kp " << p[0] << "Kd " << p[1] << "Ki " << p[2] << endl;
		cout << "Iterations needed: " << count_ <<   endl;
		found_parameter_ = true;
	}


}
