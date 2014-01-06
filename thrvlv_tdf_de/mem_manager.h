#include "tlm.h"
#include <vector>

class mem_manager: public tlm::tlm_mm_interface {
	std::vector<tlm::tlm_generic_payload*> trans_pool;

public:

	mem_manager() {
	}

	tlm::tlm_generic_payload* allocate() {
		tlm::tlm_generic_payload* trans;
		if (trans_pool.empty())
			trans = new tlm::tlm_generic_payload(this);
		else {
			trans = trans_pool.back();
			trans_pool.pop_back();
			trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		}
		return trans;
	}

	void free(tlm::tlm_generic_payload* trans) {
		trans->reset();			//
		trans_pool.push_back(trans);
	}

};

