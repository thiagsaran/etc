// int_phy.h
// internal to physical value converter

#ifndef INT_PHY_H
#define INT_PHY_H

#include <systemc-ams>

SCA_TDF_MODULE(int_phy)
{
public:

	sca_tdf::sca_in<double> inThrvlv_rSet;
	sca_tdf::sca_in<double> inThrvlv_rActPos;
	sca_tdf::sca_out<double> outThrvlv_rSet;
	sca_tdf::sca_out<double> outThrvlv_rActPos;

	int_phy( sc_core::sc_module_name nm )
	: inThrvlv_rSet("inThrvlv_rSet"),inThrvlv_rActPos("inThrvlv_rActPos"),outThrvlv_rSet("outThrvlv_rSet"),outThrvlv_rActPos("outThrvlv_rActPos")
	{
		tmpThrvlv_rSet=tmpThrvlv_rActPos=0;
	}

	void set_attributes() {
		//out.set_timestep(t_step);
		accept_attribute_changes();
	}

	void processing() {
		tmpThrvlv_rSet = inThrvlv_rSet.read();
		tmpThrvlv_rSet = ((tmpThrvlv_rSet - 2) / 8) * 100;
		if(tmpThrvlv_rSet<0)
			tmpThrvlv_rSet=0;
		outThrvlv_rSet.write(tmpThrvlv_rSet);

		tmpThrvlv_rActPos = inThrvlv_rActPos.read();
		tmpThrvlv_rActPos = ((tmpThrvlv_rActPos - 2) / 8) * 100;
		if(tmpThrvlv_rActPos<0)
			tmpThrvlv_rActPos=0;
		outThrvlv_rActPos.write(tmpThrvlv_rActPos);

	}

private:
	double tmpThrvlv_rSet,tmpThrvlv_rActPos;
};

#endif // INT_PHY_H
/*
 * Local Variables:
 * mode: C++
 * End:
 */
