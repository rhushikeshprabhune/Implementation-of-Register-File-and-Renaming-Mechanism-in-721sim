#include "pipeline.h"
#include "trap.h"
#include "mmu.h"


void pipeline_t::retire(size_t& instret) {
   bool head_valid;
   bool completed, exception, load_viol, br_misp, val_misp, load, store, branch, amo, csr;
   reg_t offending_PC;

   bool amo_success;
   trap_t *trap = NULL; // Supress uninitialized warning.


   // FIX_ME #17a
   // Call the precommit() function of the renamer module.  This tells the renamer module to return
   // the state of the instruction at the head of its Active List, if any.  The renamer does not
   // take any action, itself.
   // Moreover:
   // * The precommit() function returns 'true' if there is a head instruction (active list not empty)
   //   and 'false' otherwise (active list empty).
   // * The precommit() function also modifies arguments that THIS function must examine and act upon.
   //
   // Tips:
   // 1. Call the renamer module's precommit() function with the following arguments in this order:
   //    'completed', 'exception', 'load_viol', 'br_misp', 'val_misp', 'load', 'store', 'branch', 'amo', 'csr', 'offending_PC'.
   //    These are local variables, already declared above.
   // 2. Use the return value of the precommit() function call, to set the already-declared local
   //    variable 'head_valid'.
   // 3. Study the code that follows the code that you added, and simply note the following observations:
   //    * The retire stage only does something if the renamer module's precommit() function signals
   //      a non-empty active list and that the head instruction is complete: "if (head_valid && completed)".
   //    * If the completed head instruction is not an exception -- "if (!exception)" -- some
   //      additional processing is needed for loads and stores (LSU.commit()), atomic memory operations
   //      (execute_amo()), and system instructions (execute_csr()).  Note that a store, amo, or csr instruction
   //      may set the exception flag at this point due to deferring their final execution to the retire stage.
   //    * If, even after the additional processing of the above-deferred instructions, the completed head instruction
   //      is not an exception or load violation -- "if (!exception && !load_viol)" -- it is committed.
   //      Branches are finalized in the branch prediction unit.  Committed results are checked against
   //      the functional simulator via the checker() function call.  Serializing instructions, such as
   //      atomics and system instructions, dictate that the pipeline be squashed.
   //      If the simulator ever sets the br_misp and val_misp flags, specifically due to implementing
   //      "approach #1 recovery" for branch and value mispredictions, then these are also included
   //      as conditions for a complete pipeline squash.
   //    * Alternatively, if the completed head instruction is not an exception but a load violation --
   //      "else if (!exception && load_viol)" -- all instructions including the load instruction are squashed.
   //    * Alternatively, if the completed head instruction is an exception, the trap is taken and the pipeline
   //      is squashed including the offending instruction.
   head_valid = REN->precommit(completed, exception, load_viol, br_misp, val_misp, load, store, branch, amo, csr, offending_PC);


   if (head_valid && completed) {    // AL head exists and completed

      // Sanity checks of the 'amo' and 'csr' flags.
      assert(!amo || IS_AMO(PAY.buf[PAY.head].flags));
      assert(!csr || IS_CSR(PAY.buf[PAY.head].flags));

      // If no exception (yet):
      // 1. If the instruction is a load or store, signal the LSU to commit the load or store.
      //    The store may raise an exception here.
      // 2. If the instruction is an atomic memory operation (read-modify-write a memory address), execute it now.
      //    The atomic may raise an exception here.
      // 3. If the instruction is a csr instruction, execute it now.
      //    The csr instruction may raise an exception here.
      if (!exception) {
         if (load || store) {
            assert(load != store);   // Make sure that the same instruction does not have both flags set/cleared.
            exception = LSU.commit(load, amo, amo_success);
            if (amo && store)
               assert(amo_success);  // Assert store-conditionals (SC) are successful.
         }
         else if (amo) {
            exception = execute_amo();
         }
         else if (csr) {
            exception = execute_csr();
         }

         if (exception)
	    REN->set_exception(PAY.buf[PAY.head].AL_index);
      }

      if (!exception && !load_viol) {
	 //
         // FIX_ME #17b
	 // Commit the instruction at the head of the active list.
	 //
    REN->commit();

         // If the committed instruction is a branch, signal the branch predictor to commit its oldest branch.
         if (branch && !PERFECT_BRANCH_PRED) {
	    // TODO (ER): Change the branch predictor interface as follows: BP.commit().
            BP.verify_pred(PAY.buf[PAY.head].pred_tag, PAY.buf[PAY.head].c_next_pc, false);
         }

         // If FP op, cheat and copy the fflags from the functional simulator.
         // TODO: fflags should be (and can be) generated by the ALU. This was done to expedite porting of 721sim to RISCV from PISA.
         if (IS_FP_OP(PAY.buf[PAY.head].flags)) {
	    db_t *actual = pipe->peek(PAY.buf[PAY.head].db_index);	// Pointer to corresponding instruction in the functional simulator.
            get_state()->fflags = actual->a_state->fflags;
         }

	 // Check results.
	 checker();

	 // Keep track of the number of retired instructions.
	 num_insn++;
         instret++;
	 inc_counter(commit_count);
	 if (PAY.buf[PAY.head].split && PAY.buf[PAY.head].upper)
            num_insn_split++;

	 // Cases of complete pipeline squash after the head instruction.
	 // 1. Atomic memory operation.
	 // 2. System instruction.
	 // 3. Mispredicted branch for which "approach #1 recovery" is configured.
	 // 4. Value-mispredicted instruction for which "approach #1 recovery" is configured.
         if (amo || csr || br_misp || val_misp) {
            insn_t inst = PAY.buf[PAY.head].inst;
	    reg_t next_inst_pc;
            if ((inst.funct3() == FN3_SC_SB) && (inst.funct12() == FN12_SRET))  // SRET instruction.
               next_inst_pc = state.epc;
	    else if (br_misp)
               next_inst_pc = PAY.buf[PAY.head].c_next_pc;
	    else
	       next_inst_pc = INCREMENT_PC(PAY.buf[PAY.head].pc);

            // The head instruction was already committed above (fix #17b).
	    // Squash all instructions after it.
            squash_complete(next_inst_pc);
            inc_counter(recovery_count);

	    // Pop the instruction from PAY.
	    if (!PAY.buf[PAY.head].split) PAY.pop();
	    PAY.pop();

            // Flush PAY.
            PAY.clear();
         }
         else {
	    // Pop the instruction from PAY.
	    if (!PAY.buf[PAY.head].split) PAY.pop();
	    PAY.pop();
         }
      }
      else if (!exception && load_viol) {
	 // This is a mispredicted load owing to speculative memory disambiguation (not value prediction).
	 // Therefore the load is incorrect and not committed.
         assert(load);

         // If the simple sticky-bit memory dependence predictor is enabled,
         // add the offending load to the predictor.
	 if (MEM_DEP_PRED) {
	    //assert(MDP.find(PAY.buf[PAY.head].pc) == MDP.end());
	    MDP[PAY.buf[PAY.head].pc] = true;
	 }

         // Full squash, including the mispredicted load, and restart fetching from the load.
         squash_complete(offending_PC);
         inc_counter(recovery_count);

         // Flush PAY.
         PAY.clear();
      }
      else {   // exception
         trap = PAY.buf[PAY.head].trap;

         // CSR exceptions are micro-architectural exceptions and are
         // not defined by the ISA. These must be handled exclusively by
         // the micro-arch and is different from other exceptions specified
         // in the ISA.
         // This is a serialize trap - Refetch the CSR instruction
         reg_t jump_PC;
         if (trap->cause() == CAUSE_CSR_INSTRUCTION) {
            jump_PC = offending_PC;
         } 
         else {
            jump_PC = take_trap(*trap, offending_PC);
         }

         // Keep track of the number of retired instructions.
	 instret++;
	 num_insn++;	
         inc_counter(commit_count);
         inc_counter(exception_count);

         // Compare pipeline simulator against functional simulator.
         checker();

         // Squash the pipeline.
         squash_complete(jump_PC);
         inc_counter(recovery_count);

         // Flush PAY.
         PAY.clear();
      }
   }
}


bool pipeline_t::execute_amo() {
   unsigned int index = PAY.head;
   insn_t inst = PAY.buf[index].inst;
   reg_t read_amo_value = 0xdeadbeef;
   bool exception = false;

   try {
      if (inst.funct3() == FN3_AMO_W) {
         read_amo_value = mmu->load_int32(PAY.buf[index].A_value.dw);
         uint32_t write_amo_value;
         switch (inst.funct5()) {
            case FN5_AMO_SWAP:
               write_amo_value = PAY.buf[index].B_value.dw;
               break;
            case FN5_AMO_ADD:
               write_amo_value = PAY.buf[index].B_value.dw + read_amo_value;
               break;
            case FN5_AMO_XOR:
               write_amo_value = PAY.buf[index].B_value.dw ^ read_amo_value;
               break;
            case FN5_AMO_AND:
               write_amo_value = PAY.buf[index].B_value.dw & read_amo_value;
               break;
            case FN5_AMO_OR:
               write_amo_value = PAY.buf[index].B_value.dw | read_amo_value;
               break;
            case FN5_AMO_MIN:
               write_amo_value = std::min(int32_t(PAY.buf[index].B_value.dw), int32_t(read_amo_value));
               break;
            case FN5_AMO_MAX:
               write_amo_value = std::max(int32_t(PAY.buf[index].B_value.dw), int32_t(read_amo_value));
               break;
            case FN5_AMO_MINU:
               write_amo_value = std::min(uint32_t(PAY.buf[index].B_value.dw), uint32_t(read_amo_value));
               break;
            case FN5_AMO_MAXU:
               write_amo_value = std::max(uint32_t(PAY.buf[index].B_value.dw), uint32_t(read_amo_value));
               break;
            default:
               assert(0);
               break;
         }
         mmu->store_uint32(PAY.buf[index].A_value.dw, write_amo_value);
      }
      else if (inst.funct3() == FN3_AMO_D) {
         read_amo_value = mmu->load_int64(PAY.buf[index].A_value.dw);
         reg_t write_amo_value;
         switch (inst.funct5()) {
            case FN5_AMO_SWAP:
               write_amo_value = PAY.buf[index].B_value.dw;
               break;
            case FN5_AMO_ADD:
               write_amo_value = PAY.buf[index].B_value.dw + read_amo_value;
               break;
            case FN5_AMO_XOR:
               write_amo_value = PAY.buf[index].B_value.dw ^ read_amo_value;
               break;
            case FN5_AMO_AND:
               write_amo_value = PAY.buf[index].B_value.dw & read_amo_value;
               break;
            case FN5_AMO_OR:
               write_amo_value = PAY.buf[index].B_value.dw | read_amo_value;
               break;
            case FN5_AMO_MIN:
               write_amo_value = std::min(int64_t(PAY.buf[index].B_value.dw), int64_t(read_amo_value));
               break;
            case FN5_AMO_MAX:
               write_amo_value = std::max(int64_t(PAY.buf[index].B_value.dw), int64_t(read_amo_value));
               break;
            case FN5_AMO_MINU:
               write_amo_value = std::min(PAY.buf[index].B_value.dw, read_amo_value);
               break;
            case FN5_AMO_MAXU:
               write_amo_value = std::max(PAY.buf[index].B_value.dw, read_amo_value);
               break;
            default:
               assert(0);
               break;
         }
         mmu->store_uint64(PAY.buf[index].A_value.dw, write_amo_value);
      }
      else {
         assert(0);
      }
   }
   catch (mem_trap_t& t) {
      exception = true;
      switch (t.cause()) {
         case CAUSE_FAULT_STORE:
            PAY.buf[index].trap = new trap_store_access_fault(t.get_badvaddr());
            break;
         case CAUSE_MISALIGNED_STORE:
            PAY.buf[index].trap = new trap_store_address_misaligned(t.get_badvaddr());
            break;
         default:
            assert(0);
            break;
      }
   }

   // Record the loaded value in the payload buffer for checking purposes.
   PAY.buf[index].C_value.dw = read_amo_value;

   // Write the loaded value to the destination physical register.
   assert(PAY.buf[index].C_valid);
   REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);

   return(exception);
}


bool pipeline_t::execute_csr() {
   unsigned int index = PAY.head;
   insn_t inst = PAY.buf[index].inst;
   pipeline_t *p = this; // *p is assumed by the validate_csr and require_supervisor macros.
   int csr;
   bool exception = false;

   // CSR instructions:
   // 1. read the addressed CSR and write its old value into a destination register,
   // 2. modify the old value to get a new value, and
   // 3. write the new value into the addressed CSR.
   reg_t old_value;
   reg_t new_value;

   try {
      if (inst.funct3() != FN3_SC_SB) {
         switch (inst.funct3()) {
            case FN3_CLR:
               csr = validate_csr(PAY.buf[index].CSR_addr, true);
	       old_value = get_pcr(csr);
               new_value = (old_value & ~PAY.buf[index].A_value.dw);
               set_pcr(csr, new_value);
               break;
            case FN3_RW:
               csr = validate_csr(PAY.buf[index].CSR_addr, true);
	       old_value = get_pcr(csr);
               new_value = PAY.buf[index].A_value.dw;
               set_pcr(csr, new_value);
               break;
            case FN3_SET:
               csr = validate_csr(PAY.buf[index].CSR_addr, (PAY.buf[index].A_log_reg != 0));
	       old_value = get_pcr(csr);
               new_value = (old_value | PAY.buf[index].A_value.dw);
               set_pcr(csr, new_value);
               break;
            case FN3_CLR_IMM:
               csr = validate_csr(PAY.buf[index].CSR_addr, true);
	       old_value = get_pcr(csr);
               new_value = (old_value & ~(reg_t)PAY.buf[index].A_log_reg);
               set_pcr(csr, new_value);
               break;
            case FN3_RW_IMM:
               csr = validate_csr(PAY.buf[index].CSR_addr, true);
	       old_value = get_pcr(csr);
               new_value = (reg_t)PAY.buf[index].A_log_reg;
               set_pcr(csr, new_value);
               break;
            case FN3_SET_IMM:
               csr = validate_csr(PAY.buf[index].CSR_addr, true);
	       old_value = get_pcr(csr);
               new_value = (old_value | (reg_t)PAY.buf[index].A_log_reg);
               set_pcr(csr, new_value);
               break;
            default:
               assert(0);
               break;
         }
      }
      else if (inst.funct12() == FN12_SRET) {
         // This is a macro defined in decode.h.
         // This will throw a privileged_instruction trap if processor not in supervisor mode.
         require_supervisor; 
         csr = validate_csr(PAY.buf[index].CSR_addr, true);
         old_value = get_pcr(csr);
         new_value = ((old_value & ~(SR_S | SR_EI)) | ((old_value & SR_PS) ? SR_S : 0) | ((old_value & SR_PEI) ? SR_EI : 0));
         set_pcr(csr, new_value);
      }
      else {
         // SCALL and SBREAK.
         // These skip the IQ and execution lanes (completed in Dispatch Stage).
         assert(0);
      }

      if (PAY.buf[index].C_valid) {
         // Write the result (old value of CSR) to the payload buffer for checking purposes.
         PAY.buf[index].C_value.dw = old_value;
         // Write the result (old value of CSR) to the physical destination register.
         REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);
      }
   }
   catch (trap_t& t) {
      exception = true;
      switch (t.cause()) {
         case CAUSE_PRIVILEGED_INSTRUCTION:
            PAY.buf[index].trap = new trap_privileged_instruction();
            break;
         case CAUSE_FP_DISABLED:
            PAY.buf[index].trap = new trap_fp_disabled();
            break;
         default:
            fflush(0);
            assert(0);
            break;
      }
   }
   catch (serialize_t& s) {
      exception = true;
      PAY.buf[index].trap = new trap_csr_instruction();
   }

   return(exception);
}

