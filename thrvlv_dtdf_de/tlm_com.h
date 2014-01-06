/*
 * tlm_com.h
 *
 *  Created on: Nov 4, 2013
 *      Author: thiag
 */

#ifndef TLM_COM_H_
#define TLM_COM_H_

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "systemc.h"
#include <systemc-ams>

//#define DEBUG_PNT

static const double adc_1lsb = 0.305175781e-3;
enum {
	SIZE = 16
};
int adc_data_tns[SIZE];
int adc_data_rcp[SIZE];

int pwm_data_tns[SIZE];
int pwm_data_rcp[SIZE];

union d1 {
	double data_real;
	char data_char[8];
};
union d2 {
	double err_real;
	char err_char[8];
};
struct trans_conv {
	d1 pwm;
	d2 err;
};

trans_conv pwm_data_ts, pwm_data_rs;

struct adc_data_r: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_target_socket<adc_data_r> socket;

	SC_CTOR(adc_data_r)
	: socket("socket")
	{
		// Register callback for incoming b_transport interface method call
		socket.register_b_transport(this, &adc_data_r::b_transport);

		// Initialize adc_data_r with random data
		for (int i = 0; i < SIZE; i++)
		adc_data_rcp[i] = 0xAA000000;
	}

	// TLM-2 blocking transport method
	virtual void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
		tlm::tlm_command cmd = trans.get_command();
		sc_dt::uint64 adr = trans.get_address();
		unsigned char* ptr = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned char* byt = trans.get_byte_enable_ptr();
		unsigned int wid = trans.get_streaming_width();

		// Obliged to check address range and check for unsupported features,
		//   i.e. byte enables, streaming, and bursts
		// Can ignore DMI hint and extensions
		// Using the SystemC report handler is an acceptable way of signalling an error

		if (adr >= sc_dt::uint64(SIZE) || byt != 0 || len > 16 || wid < len)
			SC_REPORT_ERROR("TLM-2",
					"Target does not support given generic payload transaction");

		// Obliged to implement read and write commands
		if (cmd == tlm::TLM_READ_COMMAND)
			memcpy(ptr, &adc_data_rcp[adr], len);
		else if (cmd == tlm::TLM_WRITE_COMMAND)
			memcpy(&adc_data_rcp[adr], ptr, len);

#ifdef DEBUG_PNT
		cout << "trans reached in adc target = { " << cmd << ", adr " << hex
		<< adr << " } , data = " << dec << adc_data_rcp[adr]
		<< " at time " << sc_time_stamp() << " delay = " << delay
		<< endl;
#endif
		// Obliged to set response status to indicate successful completion
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}

};

struct adc_data_t: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_initiator_socket<adc_data_t> socket;
	char tmp_cmd;
	sc_in<sc_bv<16> > meas_bv, ref_bv;
	sc_in<bool> clk;

	void thread_process() {
		// TLM-2 generic payload transaction, reused across calls to b_transport

		tlm::tlm_generic_payload *trans = new tlm::tlm_generic_payload;
		sc_time delay = sc_time(10, SC_NS);

		// Generate a random sequence of reads and writes
		for (int i = 0; i < 16; i += 4) {

			tlm::tlm_command cmd = static_cast<tlm::tlm_command>(1);
			if (cmd == tlm::TLM_WRITE_COMMAND) {
				adc_data_tns[0] = meas_bv.read().to_uint();
				adc_data_tns[1] = ref_bv.read().to_uint();
			}
			// Initialize 8 out of the 10 attributes, byte_enable_length and extensions being unused
			trans->set_command(cmd);
			trans->set_address(i);
			trans->set_data_ptr(
					reinterpret_cast<unsigned char*>(&adc_data_tns));
			trans->set_data_length(8);
			trans->set_streaming_width(8); // = data_length to indicate no streaming
			trans->set_byte_enable_ptr(0); // 0 indicates unused
			trans->set_dmi_allowed(false); // Mandatory initial value
			trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE); // Mandatory initial value

			socket->b_transport(*trans, delay);  // Blocking transport call

			// Initiator obliged to check response status and delay
			if (trans->is_response_error())
				SC_REPORT_ERROR("TLM-2", "Response error from b_transport");
			if (cmd == 1)
				tmp_cmd = 'W';
			else if (cmd == 2)
				tmp_cmd = 'R';
			else
				tmp_cmd = 'I';
#ifdef DEBUG_PNT
			cout << "trans at adc ini = { " << tmp_cmd << ", " << hex << i
			<< " } , data = " << dec << adc_data_tns[0] << " at time "
			<< sc_time_stamp() << " delay = " << delay << endl;
#endif
			// Realize the delay annotated onto the transport call
			//wait(delay);
		}

	}

	SC_CTOR(adc_data_t)
	: socket("socket")  // Construct and name socket
	{

		SC_METHOD (thread_process);
		sensitive << clk.pos()<< clk.neg();
		dont_initialize();
	}

};

struct pwm_data_r: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_target_socket<pwm_data_r> socket;
	sc_out<double> pwm_ctrl;
	sc_out<double> curr_err;

	SC_CTOR(pwm_data_r): socket("socket")
	{
		// Register callback for incoming b_transport interface method call
		socket.register_b_transport(this, &pwm_data_r::b_transport);

		// Initialize pwm_data_r with random data
		for (int i = 0; i < SIZE; i++)
		pwm_data_rcp[i] = 0xAA000000;
	}

	// TLM-2 blocking transport method
	virtual void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
		tlm::tlm_command cmd = trans.get_command();
		sc_dt::uint64 adr = trans.get_address();
		unsigned char* ptr = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned char* byt = trans.get_byte_enable_ptr();
		unsigned int wid = trans.get_streaming_width();

		// Obliged to check address range and check for unsupported features,
		//   i.e. byte enables, streaming, and bursts
		// Can ignore DMI hint and extensions
		// Using the SystemC report handler is an acceptable way of signalling an error

		if (adr >= sc_dt::uint64(24) || byt != 0 || len > 24 || wid < len)
			SC_REPORT_ERROR("TLM-2",
					"Target does not support given generic payload transaction");

		// Obliged to implement read and write commands
		if (cmd == tlm::TLM_READ_COMMAND)
			memcpy(ptr, &pwm_data_rs.pwm.data_char, len);
		else if (cmd == tlm::TLM_WRITE_COMMAND)
			memcpy(&pwm_data_rs.pwm.data_char, ptr, len);

#ifdef DEBUG_PNT
		cout << "trans reached in pwm target = { " << cmd << ", adr " << hex
		<< adr << " } , data = " << scientific << pwm_data_rs.pwm_rl
		<< " at time " << sc_time_stamp() << " delay = " << delay
		<< endl;
#endif
		pwm_ctrl.write(pwm_data_rs.pwm.data_real);//pwm_data_rcp[0] * adc_1lsb;
		curr_err.write(pwm_data_rs.err.err_real);
#ifdef DEBUG_PNT
		cout << "\n PWM cntrl " << pwm_ctrl << endl;
#endif
		// Obliged to set response status to indicate successful completion
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}

};

struct pwm_data_t: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_initiator_socket<pwm_data_t> socket;
	sc_in<bool> clk;

	void thread_process() {
		// TLM-2 generic payload transaction, reused across calls to b_transport

		tlm::tlm_generic_payload *trans = new tlm::tlm_generic_payload;
		sc_time delay = sc_time(10, SC_NS);

		// Generate a random sequence of reads and writes
		for (int i = 0; i < 24; i += 8) {

			tlm::tlm_command cmd = static_cast<tlm::tlm_command>(1);
			//if (cmd == tlm::TLM_WRITE_COMMAND) pwm_data = meas_bv.to_uint();

			// Initialize 8 out of the 10 attributes, byte_enable_length and extensions being unused
			trans->set_command(cmd);
			trans->set_address(i);
			trans->set_data_ptr(
					reinterpret_cast<unsigned char*>(&pwm_data_ts.pwm.data_char));
			trans->set_data_length(16);
			trans->set_streaming_width(16); // = data_length to indicate no streaming
			trans->set_byte_enable_ptr(0); // 0 indicates unused
			trans->set_dmi_allowed(false); // Mandatory initial value
			trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE); // Mandatory initial value

			socket->b_transport(*trans, delay);  // Blocking transport call

			// Initiator obliged to check response status and delay
			if (trans->is_response_error())
				SC_REPORT_ERROR("TLM-2", "Response error from b_transport");

#ifdef DEBUG_PNT
			cout << "trans pwm ini = { " << tmp_cmd << ", " << hex << i
			<< " } , data = " << pwm_data_ts.pwm_rl << " at time "
			<< sc_time_stamp() << " delay = " << delay << endl;
#endif
			// Realize the delay annotated onto the transport call
			//wait(delay);
		}

	}

	SC_CTOR(pwm_data_t)
	: socket("socket")  // Construct and name socket
	{
		SC_METHOD (thread_process);
		sensitive << clk.neg()<<clk.pos();
		dont_initialize();
	}

};

#endif /* TLM_COM_H_ */

