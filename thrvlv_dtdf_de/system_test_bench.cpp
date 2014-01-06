// system_test_bench.cpp
//Autor:Thiyagarajan Purusothaman
// TODO: improve parametrization of pi_ctrl1 and thrvlv_motor1 -- especially the omega parameters (2 pi f).

#define SC_INCLUDE_DYNAMIC_PROCESSES
#define CLOCK_PERIOD_REF 10000 // clock period in nanoseconds
#define CLOCK_PERIOD_DE  10000 // clock period in nanoseconds
#include <systemc.h>
#include <systemc-ams>
#include <tuv_ams_library/tuv_ams_library.h>

#include <ctime>
#include "ref_src.h"
#include "converters.h"
#include "int_phy.h"
#include "cpu.h"
#include "pwm.h"
#include "throttle_driver.h"

using namespace TUV_ams_lib::bb;

int sc_main(int argc, char* argv[]) {
	using sca_core::sca_time;
	sc_core::sc_set_time_resolution(1.0, sc_core::SC_FS);
	clock_t s, e;
	s = clock();
	// ref_src
	const double ref_val = 2;
	const sca_time t_step = sca_core::sca_time(CLOCK_PERIOD_REF,
			sc_core::SC_NS);
	// pi_ctrl1

	//double kp = 1.0 / 15.0;	 // original value: 2.0
	//double ki = 2.0 * M_PI * 2.0; 	// original value: 5.0

	// pwm1
	const double v0 = 0.0;
	const double v1 = 1.0;
	const sca_time t_period = sca_time(100 * 0.01, sc_core::SC_MS); // original value: 255 * 0.05 ms
	const sca_time t_ramp = sca_time(0.05, sc_core::SC_MS); // original value: 0.05 ms
	// thrvlv_motor1
	const double h0 = 15.0;                  // original value: 255.0
	const double omega0 = 2.0 * M_PI * 10.0; // original value: 20.0

	sca_tdf::sca_signal<double> iref, ierr, imeas, ictrl, vdrv, Thrvlv_rActPos,
			Thrvlv_rSet;

	sc_clock clk1("clk1", CLOCK_PERIOD_DE, SC_NS);

	/* ADC Signals - SystemC-AMS*/
	sca_tdf::sca_signal<double> in_ad;	       		   // Input port  in TDF MoC
	sca_tdf::sca_signal<sc_bv<16> > out_ad;            // Output port in TDF MoC
	sc_core::sc_signal<sc_bv<16> > ad_out_de1, ad_out_de2; // Output port in DE MoC
	sc_core::sc_signal<bool> EOC_sig1, EOC_sig2, cklout1, cv2;
	sc_signal<double> iref_sc, imeas_sc, ictrl_sc, ierr_sc;
	sca_tdf::sca_signal<sc_bv<16> > iref_bv;

	/*setpoint interface for throttle valve simulation
	 * the imeas signal is given as dummy input to create
	 * a dynamic TDF cluster of this module for simulation benefits
	 */
	ref_src ref_src1("ref_src1", ref_val, t_step);
	ref_src1.in(imeas);
	ref_src1.out(iref);

	adc<16> adc_1("adc_1", 10, 0.0, 0.0, 0); // Module_name , Reference Voltage, gain_e, offset_e, nl_m
	adc_1.in(iref);
	adc_1.out(iref_bv);

	conv_tdf_sc<16> tdf_sc1("tdf_sc1");
	tdf_sc1.inTDF(iref_bv);
	/*END SystemC-AMS*/
	//SystemC Discrete Domain Signals->interface using TLM technique
	tdf_sc1.outDE(ad_out_de1);
	tdf_sc1.EOC(EOC_sig1);

	/*conversion from tdf signal to DE signal for interfacing to
	 * PI control algorithm in Systemc
	 */

	cpu process1("process1");
	process1.clk(clk1);
	process1.clkout(cklout1);
	// this error signal for system analysis and plotting
	//process1.error(ierr_sc);

	pwm_data_t *pwm_ini = new pwm_data_t("pwm_data_t");
	pwm_data_r *pwm_tgt = new pwm_data_r("pwm_data_r");
	pwm_ini->clk(clk1);
	pwm_ini->socket.bind(pwm_tgt->socket);
	pwm_tgt->pwm_ctrl(ictrl_sc);
	pwm_tgt->curr_err(ierr_sc);

	/*converting the DE to TDF for interfacing the control output to PWM generation
	 * ierr is converted only for plotting and system analysis
	 */

	conv_sc_tdf_real conv_sc_tdf_real1("conv_sc_tdf_real1");
	conv_sc_tdf_real1.inDE(ictrl_sc);
	conv_sc_tdf_real1.outTDF(ictrl);

	conv_sc_tdf_real conv_sc_tdf_real2("conv_sc_tdf_real2");
	conv_sc_tdf_real2.inDE(ierr_sc);
	conv_sc_tdf_real2.outTDF(ierr);

	/*PWM generation for interfacing with throttle motor model in ELN MoC
	 * For virtual protypes with more details this module can be modified
	 * with Timer based interface in Microcontroller and can be triggered
	 * as MCU pin toggling
	 */

	pwm pwm1("pwm1", v0, v1, t_period, t_ramp);
	pwm1.in(ictrl);
	pwm1.out(vdrv);

	/*throttle motor and its associated dynamics needs much improvement in next version
	 *Spring constant of throttle blade,opening position and load current relation needs
	 *to be modeled in next version
	 */

	thrvlv thrvlv_motor1("thrvlv_motor1", h0, omega0);
	thrvlv_motor1.in(vdrv);
	thrvlv_motor1.curr_out(imeas);

	/*ADC to sense the throttle position
	 */

	adc<16> adc_2("adc_2", 10, 0.0, 0.0, 0); // Module_name , Reference Voltage, gain_e, offset_e, nl_m
	adc_2.in(imeas);
	adc_2.out(out_ad);

	/* Instantiating and connecting the TDF to DE converter */
	conv_tdf_sc<16> tdf_sc2("tdf_sc");
	tdf_sc2.inTDF(out_ad);
	/*END SystemC-AMS*/
	//SystemC Discrete Domain Signals->interface using TLM technique
	tdf_sc2.outDE(ad_out_de2);
	tdf_sc2.EOC(EOC_sig2);

	adc_data_t *adc_ini = new adc_data_t("adc_data_t");
	adc_data_r *adc_tgt = new adc_data_r("adc_data_r");
	adc_ini->clk(clk1);
	adc_ini->meas_bv(ad_out_de2);
	adc_ini->ref_bv(ad_out_de1);
	adc_ini->socket.bind(adc_tgt->socket);

	/*conversion from internal value to Physical value conversion*/
	int_phy int_phy1("int_phy1");
	int_phy1.inThrvlv_rSet(iref);
	int_phy1.inThrvlv_rActPos(imeas);
	int_phy1.outThrvlv_rSet(Thrvlv_rSet);
	int_phy1.outThrvlv_rActPos(Thrvlv_rActPos);

	/* ***** tracing signals ***** */
	sca_util::sca_trace_file* digital_report =
			sca_util::sca_create_vcd_trace_file("digital_trace_thrvlv");
	sca_util::sca_trace(digital_report, imeas, "imeas");
	sca_util::sca_trace(digital_report, ad_out_de2, "adc2_imeas");
	sca_util::sca_trace(digital_report, ad_out_de1, "adc1_iref");
	sca_util::sca_trace(digital_report, EOC_sig1, "EOC_sig1");
	sca_util::sca_trace(digital_report, EOC_sig2, "EOC_sig2");
	sca_util::sca_trace(digital_report, cklout1, "cklout1");
	sca_util::sca_trace(digital_report, ictrl, "ictrl");
	sca_util::sca_trace(digital_report, vdrv, "vdrv");

	//debug
	cout << "Start generating signal.vcd file" << endl;
	cout << "Finished generating signal.vcd file" << endl;

	sca_util::sca_trace_file* analog_report =
			sca_util::sca_create_tabular_trace_file("thrvlv_system_dynamics");
	sca_util::sca_trace(analog_report, iref, "iref");
	sca_util::sca_trace(analog_report, Thrvlv_rSet, "Thrvlv_rSet");
	sca_util::sca_trace(analog_report, Thrvlv_rActPos, "Thrvlv_rActPos");
	sca_util::sca_trace(analog_report, imeas, "imeas");
	sca_util::sca_trace(analog_report, ierr, "ierr");
	sca_util::sca_trace(analog_report, cklout1, "cklout1");
	sca_util::sca_trace(analog_report, EOC_sig1, "EOC_sig1");
	sca_util::sca_trace(analog_report, EOC_sig2, "EOC_sig2");
	sca_util::sca_trace(analog_report, ictrl, "ictrl");
	sca_util::sca_trace(analog_report, vdrv, "vdrv");

	//sca_util::sca_trace(tfp_tab, thrvlv_pos, "throttle_position");
	//simulation start
	sc_start(1, sc_core::SC_SEC);

	sc_core::sc_stop();

	//close tracing file
	sca_util::sca_close_vcd_trace_file(digital_report);
	sca_util::sca_close_tabular_trace_file(analog_report);

	e = clock() - s;
	printf(" MESG :: Simulation Took %ld us \n", e);

	return 0;
}
