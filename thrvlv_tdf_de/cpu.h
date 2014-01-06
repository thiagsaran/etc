#ifndef CPU_H
#define CPU_H

#include "systemc.h"
#include "tlm.h"
#include "sstream"
#include "mem_manager.h"
#include "tlm_utils/simple_initiator_socket.h"

// this class implements a TLM-Initiator, which generates random read and write
// transactions with random addresses and data of length 1 byte, and sends them to
// a TLM target through the combined initiator socket, but only by using
// _blocking_ access

struct cpu: public sc_core::sc_module {
	mem_manager my_mm;			// memory manager
	sc_in<sc_bv<16> > ref_bv;
	sc_in<sc_bv<16> > meas_bv;
	sc_in<bool> clk;

	double actual_error, error_previous, P, I, D, control_val, kp_t, ki_t, kd_t;
	static const double adc_1lsb = 0.305175781e-3;

	uint16_t current_initcount;
	uint16_t current_dutycycle;
	uint16_t current_prescaler;

	tlm_utils::simple_initiator_socket<cpu> socket;

	SC_CTOR(cpu) :
			socket("socket") {
		kp_t = 10/ 15.0;
		ki_t = 0.00001 * M_PI;
		kd_t = 0;
		error_previous = I = 0;

		current_initcount = current_dutycycle = current_prescaler = 0;

		SC_THREAD(PI_processing);
		sensitive << clk.pos();
	}

	void PI_processing() {
		wait();	// I throw in waits to model some time passing during code execution
		// very superficial though
		while (true) {
			double adc_meas, ref_meas;
			adc_meas = meas_bv.read().to_int() * adc_1lsb;// read current value
			wait();
			ref_meas = ref_bv.read().to_int() * adc_1lsb;// read current reference
			wait();
			actual_error = ref_meas - adc_meas;
			// PID
			P = actual_error; //Current error
			I += error_previous; //Sum of previous errors
			D = actual_error - error_previous;
			control_val = kp_t * P + ki_t * I + D * kd_t; //adjust Kp, Ki empirically or by using online method such as ZN

			if (control_val < 0)
				control_val = 0;
			if (control_val > 1)
				control_val = 1;
			error_previous = actual_error; //error_previous holds the previous error

			wait();
			// transmit new values via TLM
			// currently only the dutycycle is changed, prescaler & initial counter value remain constant
			uint16_t new_prescaler = 1;
			if (new_prescaler != current_prescaler) {
				current_prescaler = new_prescaler;
				send_transaction(4, current_prescaler);
				wait();
			}
			uint16_t new_initcount = 65535 - 1000;
			if (new_initcount != current_initcount) {
				current_initcount = new_initcount;
				send_transaction(0, current_initcount);
				wait();
			}
			uint16_t new_dutycycle = control_val * 65535;
			if (new_dutycycle != current_dutycycle) {
				current_dutycycle = new_dutycycle;
				send_transaction(2, current_dutycycle);
				wait();
			}
		}
	}

	void send_transaction(unsigned adr, uint16_t val) {
		tlm::tlm_generic_payload *trans; 	// create a new transaction
		trans = my_mm.allocate();// by using the memory manager; This might generate a new transaction, or
		// reuse an existing one from the pool
		trans->acquire();// Increases the transaction reference counter (via the memory manager)

		trans->set_command(tlm::TLM_WRITE_COMMAND);

		trans->set_data_length(2);
		trans->set_streaming_width(2);
		uint16_t value = val;
		trans->set_data_ptr((unsigned char*) (&value));
		trans->set_address(static_cast<sc_dt::uint64>(adr));
		trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

		// show_transaction(*trans); // this function (defined in declarations.hpp) writes some info
		// about the transaction to the console

		socket->b_transport(*trans, delay); // now we send the transaction through the socket. Note that the "->" operator
		// has to be used here, since this is actually an interface method, in contrast
		// to e.g. the socket.get_bus_width() call, which is actually a method of the
		// tlm_initiator_socket class
		// Note that both arguments are passed as reference; in particular, the target
		// can manipulate the transaction as well as the delay

		if (trans->is_response_error())	// check for a response error
		{
			std::ostringstream warn_str;    			// if there is an error,
			warn_str << trans->get_response_string();// we output the error response string with
			SC_REPORT_WARNING("TLM", warn_str.str().c_str());// SystemC's error/warning mechanism
		} else			// in case of no error, we can process the transaction,
		{   				//e.g. process the data array of a READ transaction
			if (trans->get_command() == tlm::TLM_READ_COMMAND) {
				std::cout << "Data read at " << sc_core::sc_time_stamp()
						<< ": ";   					// << std::showbase;
				for (unsigned int i = 0; i < trans->get_data_length(); i++) {
					std::cout << std::hex
							<< static_cast<int>(trans->get_data_ptr()[i])
							<< " ";
				}
				std::cout << std::endl;
			}
		}

		trans->release();// last but not least, the transaction has to be deleted. This command decreases the
		// reference count of the transaction (via the memory manager), and if it goes to 0,
		// the transaction is freed for future use. E.g. the next my_mm.allocate might reuse
		// this very transaction
	}
// small function showing the content of a transaction:
	void show_transaction(tlm::tlm_generic_payload& trans) {
		tlm::tlm_command cmd = trans.get_command();
		if (cmd == tlm::TLM_READ_COMMAND) {
			std::cout << "READ " << std::dec << trans.get_data_length()
					<< " bytes from address 0x" << std::hex
					<< trans.get_address();
		} else if (cmd == tlm::TLM_WRITE_COMMAND) {
			std::cout << "WRITE " << std::dec << trans.get_data_length()
					<< " bytes to address 0x" << std::hex
					<< trans.get_address();
		}
		std::cout << " at time " << sc_core::sc_time_stamp() << std::endl;
	}
};
#endif // CPU_H
