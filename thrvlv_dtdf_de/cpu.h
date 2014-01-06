//cpu.h
#ifndef SC_CONTROLLER_H
#define SC_CONTROLLER_H

#include "systemc.h"
#include <systemc-ams>
#include "tlm_com.h"

/*systemc based PI controller without AMS extension libraries
 *
 */

SC_MODULE(cpu) {
public:

	sc_in<bool> clk;
	sc_out<bool> clkout;
	bool tmpclk;
	double actual_error, error_previous, P, I, D, out_tmp, kp_t, ki_t, kd_t,
			ref_t;
	static const double adc_1lsb = 0.305175781e-3;
	double adc_meas, adc_ref;

	void PI_processing() {

		adc_meas = adc_data_rcp[0] * adc_1lsb;
		adc_ref = adc_data_rcp[1] * adc_1lsb;
		actual_error = adc_ref - adc_meas; //meas.read();
		/*debug output for analysing quantisation error
		 cout << "DOUBLE " << meas.read() << " " << endl;
		 cout << "bv  " << std::setprecision(9) << adc_meas << "\n" << endl;*/


		// PID
		P = actual_error; //Current error
		I += error_previous; //Sum of previous errors
		D = error_previous-actual_error; //Diff of previous errors
		out_tmp = kp_t * P + ki_t * I + D * kd_t; //adjust Kp, Ki empirically or by using online method such as ZN

		pwm_data_ts.pwm.data_real = out_tmp;
		pwm_data_ts.err.err_real = actual_error;
		tmpclk=!tmpclk;
		clkout.write(tmpclk);
		error_previous = actual_error; //error_previous holds the previous error

	}

	SC_CTOR(cpu) {

		kp_t = 40/ 15.0;
		ki_t = 0.00001 * M_PI;
		kd_t = 0.0;
		ref_t = 0;
		adc_meas = out_tmp = adc_ref = error_previous = actual_error = I = D =
				P = 0;
		tmpclk=false;
		SC_METHOD(PI_processing);
		sensitive << clk.pos() << clk.neg();
		dont_initialize();

	}

};

#endif // CPU_H
