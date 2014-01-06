// throttle_driver.h
// TODO: review default values (especially omega0)
//
// The load is limiting the maximum time step based on the
// omega0 parameter and an additional oversampling parameter.
//
// I assume that the ltf_zp functor can cope with the variable
// time step.

#ifndef THROTTLE_DRIVER_H
#define THROTTLE_DRIVER_H

#include <systemc-ams>


/*
SCA_TDF_MODULE(thrvlv)
{
  sca_tdf::sca_in<double> in;
  sca_tdf::sca_out<double> curr_out;

  thrvlv( sc_core::sc_module_name nm, double h0_ = 255.0, double omega0_ = 20.0, double oversampling_ = 100 )
  : in("in"), curr_out("curr_out"), h0(h0_), omega0(omega0_),
    t_max_step((2.0 * M_PI) / (omega0 * oversampling_), sc_core::SC_SEC),
    ltf_zp("ltf_zp"), poles(), zeros()
  {}

  void set_attributes() {

	  set_max_timestep(t_max_step);

	  accept_attribute_changes();
  }

  void change_attributes() {

  }

  void initialize()
  {
    // no zeros required
    poles(0) = sca_util::sca_complex( -omega0, 0.0 );
  }

  void processing()
  {
	  //std::cout << get_time() << " model: " << name() << std::endl;
    double tmp = ltf_zp( zeros, poles, in.read(), h0 * omega0 );
    curr_out.write(tmp);
  }

private:
  double h0, omega0;
  sca_core::sca_time t_max_step;
  sca_tdf::sca_ltf_zp ltf_zp; // Laplace transfer function
  sca_util::sca_vector<sca_util::sca_complex > poles, zeros; // poles and zeros as complex values
};

*/

SC_MODULE(thrvlv)
{
	sca_tdf::sca_in<double> in;
	sca_tdf::sca_out<double> curr_out;
	//sca_tdf::sca_out<double> thrvlv_pos;

	thrvlv( sc_core::sc_module_name nm, double h0_ = 255.0, double omega0_ = 20.0):
		in("in"), curr_out("curr_out"),
	    n1("n1"), n2("n2"), n3("n3"), gnd("gnd")
	{
		vin=new sca_eln::sca_tdf::sca_vsource("vin",h0_);
		   vin->p(n1);
		   vin->n(gnd);
		   vin->inp(in);

		imeas=new sca_eln::sca_tdf::sca_isink("imeas");
		   imeas->p(n1);
		   imeas->n(n2);
		   imeas->outp(curr_out);

		motor_r=new sca_eln::sca_r("motor_r",1.0);
		   motor_r->p(n2);
		   motor_r->n(n3);

		motor_l=new sca_eln::sca_l("motor_l",1.0/omega0_);
		   motor_l->p(n3);
		   motor_l->n(gnd);
	}

	sca_eln::sca_node n1, n2, n3;
	sca_eln::sca_node_ref gnd;

	sca_eln::sca_tdf::sca_vsource* vin;
	sca_eln::sca_tdf::sca_isink*   imeas;
	sca_eln::sca_l*                motor_l;
	sca_eln::sca_r*                motor_r;
};


#endif // THROTTLE_DRIVER_H

/*
 * Local Variables:
 * mode: C++
 * End:
 */
