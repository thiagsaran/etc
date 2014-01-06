// system_test_bench.cpp
//Autor:Thiyagarajan Purusothaman
// TODO: improve parametrization of pi_ctrl1 and thrvlv_motor1 -- especially the omega parameters (2 pi f).

#define SC_INCLUDE_DYNAMIC_PROCESSES
#define CLOCK_PERIOD 1000 // clock period in nanoseconds
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
	clock_t e,s;
	s=clock();

	// ref_src
	const double ref_val = 2;
	const sca_time t_step = sca_core::sca_time(CLOCK_PERIOD, sc_core::SC_NS);

	// thrvlv_motor1
	const double h0 = 15.0;                  // original value: 255.0
	const double omega0 = 2.0 * M_PI * 10.0; // original value: 20.0

	sca_tdf::sca_signal<double> iref, ierr, imeas, ictrl, vdrv, Thrvlv_rActPos,
			Thrvlv_rSet;
	sca_tdf::sca_signal<sc_bv<16> > iref_bv;

	sc_core::sc_signal<bool> dummy_sync;	//dummy signal to sync the pwm;

	sc_clock clk1("clk1", 1000, SC_NS);

	/* ADC Signals - SystemC-AMS*/
	sca_tdf::sca_signal<double> in_ad;	       		   // Input port  in TDF MoC
	sca_tdf::sca_signal<sc_bv<16> > out_ad;            // Output port in TDF MoC
	sc_core::sc_signal<sc_bv<16> > ad_out_de1, ad_out_de2; // Output port in DE MoC
	sc_signal<bool> EOC_sig1, EOC_sig2;
	sc_signal<double> imeas_sc, ictrl_sc;

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

	conv_tdf_sc<16> conv_tdf_sc_1("conv_tdf_sc_1");
	conv_tdf_sc_1.inTDF(iref_bv);
	conv_tdf_sc_1.outDE(ad_out_de1);
	conv_tdf_sc_1.EOC(EOC_sig1);

	cpu cpu1("cpu1");
	cpu1.ref_bv(ad_out_de1);
	cpu1.meas_bv(ad_out_de2);
	cpu1.clk(clk1);

	pwm<16, 16, 16, uint16_t, uint16_t, uint16_t> pwm1("pwm1");
	pwm1.sub.out(vdrv);

	cpu1.socket.bind(pwm1.socket);

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
	conv_tdf_sc<16> tdf_sc("tdf_sc");
	tdf_sc.inTDF(out_ad);
	tdf_sc.EOC(EOC_sig2);

	/*END SystemC-AMS*/

	//SystemC Discrete Domain Signals->interface using TLM technique
	tdf_sc.outDE(ad_out_de2);

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
	sca_util::sca_trace(analog_report, EOC_sig1, "EOC_sig1");
	sca_util::sca_trace(analog_report, EOC_sig2, "EOC_sig2");
	sca_util::sca_trace(analog_report, vdrv, "vdrv");

	//simulation start
	sc_start(1, sc_core::SC_SEC);

	sc_core::sc_stop();

	//close tracing file
	sca_util::sca_close_vcd_trace_file(digital_report);
	sca_util::sca_close_tabular_trace_file(analog_report);

	e=clock()-s;
	printf(" MESG :: Simulation Took seconds  %ld us \n",e);

	return 0;
}

