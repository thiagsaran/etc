/*
 * converters.h
 *
 * Module to convert the TDF input to DE output
 * It also implements a toggle EOC signal to the ADC
 *
 */

#include "systemc.h"
#include "systemc-ams.h"
//#include "tuv_ams_library.h"

using namespace TUV_ams_lib::bb;

template <int N>
SCA_TDF_MODULE(conv_tdf_sc)
{
public:
	sca_tdf::sca_in< sc_bv<N> > inTDF;
	sca_tdf::sca_de::sca_out<sc_bv<N> > outDE;
	sca_tdf::sca_de::sca_out<bool> EOC;
	bool EOC_tmp;

	void processing()
	{
		outDE.write(inTDF.read());
		EOC_tmp=!EOC_tmp;
		//cout<<"EOC "<<EOC_tmp<<endl;
		EOC.write(EOC_tmp);
	}

	void set_attributes()
	{
		accept_attribute_changes();
	}

	void initialize()
	{
		EOC_tmp=0;
		//EOC.write(false);
	}

	SCA_CTOR(conv_tdf_sc) :inTDF("inTDF"),outDE("outDE"),EOC("EOC") {
		EOC_tmp=0;
	};

};

SCA_TDF_MODULE(conv_tdf_sc_real)
{
public:
	sca_tdf::sca_in<double> inTDF;
	sca_tdf::sca_de::sca_out<double> outDE;
	sca_tdf::sca_de::sca_out<bool> EOC;
	bool EOC_tmp;

	void processing() {
		outDE.write(inTDF.read());
		EOC_tmp = !EOC_tmp;
		//cout<<"EOC "<<EOC_tmp<<endl;
		EOC.write(EOC_tmp);
	}

	void set_attributes() {
		set_timestep(sca_core::sca_time(CLOCK_PERIOD_REF, sc_core::SC_MS));
		//outDE.set_delay(1);
		accept_attribute_changes();
	}

	void initialize() {
		EOC_tmp = 0;

		//EOC.write(false);
	}

	SCA_CTOR(conv_tdf_sc_real) :inTDF("inTDF"),outDE("outDE"),EOC("EOC") {
		EOC_tmp=0;
	};

};

SCA_TDF_MODULE(conv_sc_tdf_real)
{
public:
	sca_tdf::sca_de::sca_in<double> inDE;
	sca_tdf::sca_out<double> outTDF;

	void processing() {
		outTDF.write(inDE.read());
	}

	void set_attributes() {
		set_timestep(sca_core::sca_time(CLOCK_PERIOD_REF, sc_core::SC_NS));
		accept_attribute_changes();
	}

	void initialize() {

	}

	SCA_CTOR(conv_sc_tdf_real) :inDE("inDE"),outTDF("outTDF") {

	};

};


