#include "pipeline.h"


void pipeline_t::squash_complete(reg_t jump_PC) {
	unsigned int i, j;

	//////////////////////////
	// Fetch Stage
	//////////////////////////
  
	//pc = offending_PC + SS_INST_SIZE;
	//pc = INCREMENT_PC(offending_PC);
	pc = jump_PC;  //Jump to the exception vector or next instruction
  LOG(fetch_log,cycle,PAY.buf[PAY.head].sequence,PAY.buf[PAY.head].pc,"Exception, Redirect to 0x%016" PRIx64 "",pc);


	next_fetch_cycle = (cycle_t)0;
	BP.flush();

  // Clear the fetch_exception flag so that instructions
  // can be fetched again. FETCH might have been stalled if 
  // an instruction fetched in the squashed path excepted.
  clear_fetch_exception();
  clear_fetch_amo();
  clear_fetch_csr();
  ifprintf(logging_on,stderr,"RETIRE: Clearing fetch_exception flag %u\n",fetch_exception);

	//////////////////////////
	// Decode Stage
	//////////////////////////

	for (i = 0; i < fetch_width; i++) {
		DECODE[i].valid = false;
	}

	//////////////////////////
	// Rename1 Stage
	//////////////////////////

	FQ.flush();

	//////////////////////////
	// Rename2 Stage
	//////////////////////////

	for (i = 0; i < dispatch_width; i++) {
		RENAME2[i].valid = false;
	}

        //
        // FIX_ME #17c
        // Squash the renamer.
        //
        REN->squash();

	//////////////////////////
	// Dispatch Stage
	//////////////////////////

	for (i = 0; i < dispatch_width; i++) {
		DISPATCH[i].valid = false;
	}

	//////////////////////////
	// Schedule Stage
	//////////////////////////

	IQ.flush();

	//////////////////////////
	// Register Read Stage
	// Execute Stage
	// Writeback Stage
	//////////////////////////

	for (i = 0; i < issue_width; i++) {
		Execution_Lanes[i].rr.valid = false;
		for (j = 0; j < Execution_Lanes[i].ex_depth; j++)
		   Execution_Lanes[i].ex[j].valid = false;
		Execution_Lanes[i].wb.valid = false;
	}

	LSU.flush();
}


void pipeline_t::resolve(unsigned int branch_ID, bool correct) {
	unsigned int i, j;

	if (correct) {
		// Instructions in the Rename2 through Writeback Stages have branch masks.
		// The correctly-resolved branch's bit must be cleared in all branch masks.

		for (i = 0; i < dispatch_width; i++) {
			// Rename2 Stage:
			CLEAR_BIT(RENAME2[i].branch_mask, branch_ID);

			// Dispatch Stage:
			CLEAR_BIT(DISPATCH[i].branch_mask, branch_ID);
		}

		// Schedule Stage:
		IQ.clear_branch_bit(branch_ID);

		for (i = 0; i < issue_width; i++) {
			// Register Read Stage:
			CLEAR_BIT(Execution_Lanes[i].rr.branch_mask, branch_ID);

			// Execute Stage:
			for (j = 0; j < Execution_Lanes[i].ex_depth; j++)
			   CLEAR_BIT(Execution_Lanes[i].ex[j].branch_mask, branch_ID);

			// Writeback Stage:
			CLEAR_BIT(Execution_Lanes[i].wb.branch_mask, branch_ID);
		}
	}
	else {
		// Squash all instructions in the Decode through Dispatch Stages.

		// Decode Stage:
		for (i = 0; i < fetch_width; i++) {
			DECODE[i].valid = false;
		}

		// Rename1 Stage:
		FQ.flush();

		// Rename2 Stage:
		for (i = 0; i < dispatch_width; i++) {
			RENAME2[i].valid = false;
		}

		// Dispatch Stage:
		for (i = 0; i < dispatch_width; i++) {
			DISPATCH[i].valid = false;
		}

		// Selectively squash instructions after the branch, in the Schedule through Writeback Stages.

		// Schedule Stage:
		IQ.squash(branch_ID);

		for (i = 0; i < issue_width; i++) {
			// Register Read Stage:
			if (Execution_Lanes[i].rr.valid && BIT_IS_ONE(Execution_Lanes[i].rr.branch_mask, branch_ID)) {
				Execution_Lanes[i].rr.valid = false;
			}

			// Execute Stage:
			for (j = 0; j < Execution_Lanes[i].ex_depth; j++) {
			   if (Execution_Lanes[i].ex[j].valid && BIT_IS_ONE(Execution_Lanes[i].ex[j].branch_mask, branch_ID)) {
				Execution_Lanes[i].ex[j].valid = false;
			   }
			}

			// Writeback Stage:
			if (Execution_Lanes[i].wb.valid && BIT_IS_ONE(Execution_Lanes[i].wb.branch_mask, branch_ID)) {
				Execution_Lanes[i].wb.valid = false;
			}
		}
	}
}
