#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0;
	this->i_error = 0;
	this->d_error = 0;

	this->sum_of_cte_error = 0.0;

	this->first_sample = true;
	this->cte_last = 0.0;

	// reset history
	std::vector<double>().swap(this->cte_error_history);
	cte_history_counter = 0;
}

void PID::UpdateError(double cte) {

	// get the time right now... for lack of a better way to get it, we'll assume this is the time the current measurements were taken
	auto now = std::chrono::high_resolution_clock::now();

	// for the first loop through we need to init last cte and last update time
	if (first_sample) {
		cte_last = cte;
		first_sample = false;

		last_update_time = now;
	}
	
	// measure elapsed time in secs - ensure no divide by zeros
	double time_delta_secs = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count() / 1000.0;
	if (time_delta_secs == 0.0)
		time_delta_secs = 0.001;

	// precompute total error and CTE differentials over time elapsed for PID calcs.

	double diff_CTE = (cte - cte_last) / time_delta_secs;

#if VANILLA_CTE

	sum_of_cte_error += cte; // uses all the history of CTE errors combined... ever

#else
	// this version uses only the sum of the last x samples' CTE error summed. This means bigger errors in the near term get a bigger 'push'
	// to correct the offset (like when going round corners!).

	const int max_history = 20;

	if (cte_error_history.size() < max_history)
	{
		cte_error_history.push_back(cte);
	}
	else
	{
		int idx = cte_history_counter % cte_error_history.size();

		cte_error_history[idx] = cte;
	}

	cte_history_counter++;


	sum_of_cte_error = 0.0;
	for (size_t i = 0; i < cte_error_history.size(); i++)
	{
		sum_of_cte_error += cte_error_history[i];
	}

#endif

	// cache the params used to calculate the steering angle in the TotalError() func
	p_error = cte;
	d_error = diff_CTE;
	i_error = sum_of_cte_error;

	// cache for next loop through so we can calc the difference
	cte_last = cte;

	// cache this update time for the next CTE differential computation (next loop through)
	last_update_time = now;
}

double PID::TotalError() {
	return -(Kp * p_error - Kd * d_error - Ki * i_error);
}

/*
double twiddle(double tol = 0.2) {
	// Don't forget to call `make_robot` before every call of `run`!
		p = [0, 0, 0]
		dp = [1, 1, 1]
		robot = make_robot()
		x_trajectory, y_trajectory, best_err = run(robot, p)

		// TODO: twiddle loop here
		it = 0

		while sum(dp) > tol :

	it += 1

		print("Iteration {}, best error = {}".format(it, best_err))

		for i in range(len(p)) :

			p[i] += dp[i]

			robot = make_robot()
			x_trajectory, y_trajectory, err = run(robot, p)

			if err < best_err:
	best_err = err
		dp[i] *= 1.1
			else:
	p[i] -= 2 * dp[i]

		if err < best_err :
			best_err = err
			dp[i] *= 1.1
		else:
	p[i] += dp[i]
		dp[i] *= 0.9

		return p, best_err
}
*/

