// pwm.h
// pwm module which is controlled with registers which can be accessed via TLM
// The pulses are output via a double TDF port
// the TLM-part is an SC_MODULE, which contains a TDF-module for the TDF part
// main reason: TDF-modules can only contain TDF-ports or DE<->TDF converterports,
//		and a TLM socket is technically a different port

#ifndef PWM_H
#define PWM_H

#include <cmath>
#include <systemc-ams>
#include "tlm_utils/simple_target_socket.h"

// TDF submodule
template <unsigned N, unsigned M, unsigned K>
SCA_TDF_MODULE(pwm_TDF)// N = bitwidth of underlying counter, M = bitwidth of dutycycle
{				// K = bitwidth of prescaler

	sca_tdf::sca_de::sca_in<bool> sync;// DE-converterport to ensure synchronisation
	sca_tdf::sca_out<double> out;// pulse out

	pwm_TDF(sc_core::sc_module_name nm,
			sc_uint<M> dutycycle_max_,
			double v0_,
			double v1_
	): out("out"),
	v0(v0_), v1(v1_), dutycycle_max(dutycycle_max_)
	{
		// we basically use as register-bits multiples of 8, should fit with the tX-types
		counterbytes = (unsigned char)ceil(N/8);
		dutycycbytes = (unsigned char)ceil(M/8);
		prescalbytes = (unsigned char)ceil(K/8);
		countval = 0;
	}

	void configure() // configure() is called everytime we access the PWM from the TLM side
	{// with new values for prescaler etc. The register-values are already set from the DE-module.

		double duty = (((double)dutycycle)/dutycycle_max);
		flipval = init_countval + (sc_uint<N>)((( (sc_uint<N>)(pow(2,N)-1) - init_countval) * duty ) + 0.5);

		// these values could be used maybe to make more use of dynamic TDF (unsused atm) :
		t_period = sca_core::sca_time((double)(CLOCK_PERIOD * prescaler * ((sc_uint<N>)(pow(2,N)-1) - init_countval)) , sc_core::SC_NS);
		t_duty = duty * t_period;
	}

	void set_attributes() {
		//does_attribute_changes();	// not atm :(
		out.set_timestep(sca_core::sca_time(CLOCK_PERIOD, sc_core::SC_NS));
	}

	void initialize() {
		prescaler = 0;	// pwm off
		outState = v0;
		//out.write(outState);
	}

	void change_attributes() {	// unused atm
		//out.set_timestep(sca_core::sca_time((double)(CLOCK_PERIOD * prescaler), sc_core::SC_NS));
	}// the idea is to call the PWM not with the period of the clock, but multiply the clock by the prescaler

	void processing()
	{
		sync.read();// sync, actually not really needed with all data rates = 1 atm
		if(prescaler == 0)// prescaler = 0 means PWM is off
		{
			outState = v0;
			countval = init_countval;
		}
		else
		{
			if(countval.to_uint() >= (pow(2,N)-1) ) // overflow imminent
			{				// by catching it before the actual overflow
				outState = v1;// we avoid runtime warnings by the sc_uint<N> overflow
				countval = init_countval;
			}
			else
			{
				countval++;
				if(countval > flipval) outState = v0;
			}
		}
		out.write(outState);
	}

	void end_of_simulation() {}

	double v0, v1;            // off-on values (in Volts)
	sc_uint<M> dutycycle_max;// duty cycle value representing 100%

	sc_uint<N> countval;// state of counter

	// values configurable during simulation ("Registers")
	sc_uint<N> init_countval;// the initial value when the counter is reset
	sc_uint<M> dutycycle;// the dutycycle register
	sc_uint<K> prescaler;// the prescaler-register

	// time values derived from the above
	sca_core::sca_time t_period;
	sca_core::sca_time t_duty;
	sc_uint<N> flipval;// count-value when we switch from 0 to 1

	double outState;// current output value;

	unsigned char counterbytes, dutycycbytes, prescalbytes;

};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// The DE-part of the pwm-Module

template <unsigned N, unsigned M, unsigned K, typename tN , typename tM, typename tK>
SC_MODULE(pwm)// N = bitwidth of underlying counter, M = bitwidth of dutycycle
{			// K = bitwidth of prescaler, tX = types to store these values

	tlm_utils::simple_target_socket<pwm> socket;// TLM socket

	sc_core::sc_signal<bool> sync;// dummy DE-signal to ensure DE-TDF syncing

	pwm_TDF<N, M, K> sub;// TDF submodule

	pwm(sc_core::sc_module_name nm,
			sc_uint<M> dutycycle_max_ = (sc_uint<M>)pow(2,M) - 1,
			double v0_ = 0.0,
			double v1_ = 12.0
	): sync("sync"), sub("sub", dutycycle_max_, v0_, v1_)
	{
		socket.register_b_transport(this, &pwm::b_transport);
		sub.sync(sync);	// connect the DE-converterport of the submodukle to the dummy DE signal
	}

	virtual void b_transport( tlm::tlm_generic_payload& trans, sc_core::sc_time& delay )
	{
		tlm::tlm_command command = trans.get_command();	// get all the parameters
		sc_dt::uint64 address = trans.get_address();// of the transaction
		unsigned char* data_ptr = trans.get_data_ptr();
		unsigned int data_len = trans.get_data_length();
		unsigned char* byte_en = trans.get_byte_enable_ptr();
		unsigned int str_wid = trans.get_streaming_width();

		delay += sc_core::sc_time(1,sc_core::SC_US);

		// check if we can process the transaction. If the parameters are such that we can't process
		// the transaction, we set the response status to the appropriate error response
		// Since we only can set one error response, we effectively impose an error priority order here
		// this order, however, is by no means mandatory

		if(byte_en != 0)// byte enable not supported error
		trans.set_response_status( tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE );
		else if(str_wid < data_len)// streaming not supported error
		trans.set_response_status( tlm::TLM_BURST_ERROR_RESPONSE );// NOTE: streaming access makes no
		// sense for memory modules
		else if(command == tlm::TLM_IGNORE_COMMAND)// this attribute might be set if the
		trans.set_response_status( tlm::TLM_COMMAND_ERROR_RESPONSE );// tranascation uses extensions; the memory
		// however can process only READ and WRITE,
		// and therefore we set the command error
		// response
		else
		{
			uint64_t adr = (uint64_t)address;
			if( (uint64_t)(adr + data_len) <= (uint64_t)(sub.counterbytes + sub.dutycycbytes + sub.prescalbytes) )// a bit sketchy...
			{
				if (command == tlm::TLM_WRITE_COMMAND)// set the register corresponding to the address
				{
					if(adr == 0) {sub.init_countval = (sc_uint<N>)(*((tN*)data_ptr)); sub.countval = sub.init_countval;}
					if(adr == sub.counterbytes ) sub.dutycycle = (sc_uint<M>)(*((tM*)data_ptr));
					if(adr == sub.counterbytes + sub.dutycycbytes) sub.prescaler = (sc_uint<K>)(*((tK*)data_ptr));
					sync.write(!sync.read());	// not sure I need that
					sub.configure();
				}
				else if (command == tlm::TLM_READ_COMMAND)// WRITE => copy data from transaction
				{
					//	...maybe later...
				}
				trans.set_response_status( tlm::TLM_OK_RESPONSE );// set OK response
			}
			else
			{
				printf("Address Error ****\n");
				trans.set_response_status( tlm::TLM_ADDRESS_ERROR_RESPONSE );
			}
		}
	}

	void end_of_simulation() {}

};

#endif // PWM_H
