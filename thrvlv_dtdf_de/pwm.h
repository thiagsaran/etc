// pwm.h
//
// TODO: Add sampling of duty cycle once per pulse period.
//
// Question: Is it legal to read access a TDF input signal from within
// change_attributes()? If not, we need to buffer t_duty for
// reuse in change_attributes(). This is the implemented solution.
//
// However, I assume that the current module time can be accessed from
// within change_attributes().

#ifndef PWM_H
#define PWM_H

#include <cmath>
#include <systemc-ams>

SCA_TDF_MODULE(pwm)
{
	sca_tdf::sca_in<double> in;
	sca_tdf::sca_out<double> out;

	pwm(sc_core::sc_module_name nm, double v0_ = 0.0, double v1_ = 12.0,
			const sca_core::sca_time& t_period_ = sca_core::sca_time(255.0 * 0.05, sc_core::SC_MS),
			const sca_core::sca_time& t_ramp_ = sca_core::sca_time(0.05, sc_core::SC_MS))
	: in("in"), out("out"),
	v0(v0_), v1(v1_), t_period(t_period_.to_seconds()), t_ramp(t_ramp_.to_seconds()),
	t_duty_max(t_period - 2.0 * t_ramp), t_duty(t_duty_max)
	{
		ts_cnt = 0;
		tsch_cnt = 0;
	}

	void set_attributes() {
		does_attribute_changes();
	}

	void initialize() {
		ts_last = get_timestep();
		ts_cnt = 0;
		tsch_cnt = 0;
	}

	void change_attributes() {
		double t = this->get_time().to_seconds(); // current time
		double t_pos = std::fmod(t, t_period); // time position inside pulse period

		double td_req;

		if (t_pos < t_ramp) {
			// rising edge
			td_req = t_ramp - t_pos;
		} else if (t_pos < t_ramp + t_duty) {
			// plateau
			td_req = (t_ramp + t_duty) - t_pos;
		} else if (t_pos < t_ramp + t_duty + t_ramp) {
			// falling edge
			td_req = (t_ramp + t_duty + t_ramp) - t_pos;
		} else {
			// return to initial value
			td_req = t_period - t_pos;
		}

		if (td_req > sc_core::sc_get_time_resolution().to_seconds()) {
			//std::cout << get_time() << " request new activation: " << td_req << std::endl;
			request_next_activation(td_req, sc_core::SC_SEC);
		}

	}

	void processing() {
		double t = this->get_time().to_seconds(); // current time
		double t_pos = fmod(t, t_period);   // time position inside pulse period

		// calculate and clamp duty time
		t_duty = in.read() * t_duty_max;
		if (t_duty < 0.0)
			t_duty = 0.0;
		if (t_duty > t_duty_max)
			t_duty = t_duty_max;

		double val = v0; // initial value
		if (t_pos < t_ramp) {
			// rising edge
			val = ((v1 - v0) / t_ramp) * t_pos + v0;
		} else if (t_pos < t_ramp + t_duty) {
			// plateau
			val = v1;
		} else if (t_pos < t_ramp + t_duty + t_ramp) {
			// falling edge
			val = ((v0 - v1) / t_ramp) * (t_pos - t_ramp - t_duty) + v1;
		} else {
			// return to initial value
		}
		out.write(val);

		ts_cnt++;
		if (ts_last != get_timestep()) {
			tsch_cnt++;
			ts_last = get_timestep();
		}
	}

	void end_of_simulation() {
		std::ostringstream str;
		str << "Number of timestep changes: " << tsch_cnt << " of " << ts_cnt
				<< " timesteps";
		SC_REPORT_INFO("statistic", str.str().c_str());
	}

private:
	double v0, v1;            // initial and plateau values
	double t_period, t_ramp;  // pulse period and ramp time
	double t_duty_max;        // maximum duty time
	double t_duty;            // current duty time

	sca_core::sca_time ts_last;

	unsigned long tsch_cnt;
	unsigned long ts_cnt;
};

#endif // PWM_H
/*
 * Local Variables:
 * mode: C++
 * End:
 */
