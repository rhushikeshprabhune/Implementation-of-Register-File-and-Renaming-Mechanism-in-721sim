#include "pipeline.h"
#include "debug.h"
#include "trap.h"


void pipeline_t::dispatch() {
   unsigned int i;
   unsigned int bundle_inst, bundle_load, bundle_store;
   unsigned int index;
   bool load_flag;
   bool store_flag;
   bool branch_flag;
   bool amo_flag;
   bool csr_flag;
   bool A_ready;
   bool B_ready;
   bool D_ready;
   db_t* actual;

   // Stall the Dispatch Stage if either:
   // (1) There isn't a dispatch bundle.
   // (2) There aren't enough AL entries for the dispatch bundle.
   // (3) There aren't enough IQ entries for the dispatch bundle.
   // (4) There aren't enough LQ/SQ entries for the dispatch bundle.

   // First stall condition: There isn't a dispatch bundle.
   if (!DISPATCH[0].valid) {
      return;
   }

   // FIX_ME #6
   // Second stall condition: There aren't enough AL entries for the dispatch bundle.
   // Check if the Dispatch Stage must stall due to the Active List not having sufficient entries for the dispatch bundle.
   // * If the Active List does not have enough entries for the *whole* dispatch bundle (which is always comprised of 'dispatch_width' instructions),
   //   then stall the Dispatch Stage. Stalling is achieved by returning from this function ('return').
   // * Else, don't stall the Dispatch Stage. This is achieved by doing nothing and proceeding to the next statements.
   if(REN->stall_dispatch(dispatch_width))
   {
      return;
   }


   //
   // Third and fourth stall conditions:
   // - There aren't enough IQ entries for the dispatch bundle.
   // - There aren't enough LQ/SQ entries for the dispatch bundle.
   //
   bundle_inst = 0;
   bundle_load = 0;
   bundle_store = 0;
   for (i = 0; i < dispatch_width; i++) {
      assert(DISPATCH[i].valid);
      index = DISPATCH[i].index;

      // Check IQ requirement.
      switch (PAY.buf[index].iq) {
         case SEL_IQ:
            // Increment number of instructions to be dispatched to unified IQ.
            bundle_inst++;
            break;

         case SEL_IQ_NONE:
         case SEL_IQ_NONE_EXCEPTION:
            // Skip IQ altogether.
            break;

         default:
            assert(0);
            break;
      }

      // Check LQ/SQ requirement.
      if (IS_LOAD(PAY.buf[index].flags)) {
         bundle_load++;
      }
      else if (IS_STORE(PAY.buf[index].flags)) {
         // Special cases:
         // S_S and S_D are split-stores, i.e., they are split into an addr-op and a value-op.
         // The two ops share a SQ entry to "rejoin". Therefore, only the first op should check
         // for and allocate a SQ entry; the second op should inherit the same entry.
         if (!PAY.buf[index].split_store || PAY.buf[index].upper) {
            bundle_store++;
         }
      }
   }

   // Now, check for available entries in the unified IQ and the LQ/SQ.
   if (IQ.stall(bundle_inst) || LSU.stall(bundle_load, bundle_store)) {
      return;
   }


   //
   // Making it this far means we have all the required resources to dispatch the dispatch bundle.
   //
   for (i = 0; i < dispatch_width; i++) {
      assert(DISPATCH[i].valid);
      index = DISPATCH[i].index;

      // Choose an execution lane for the instruction.
      PAY.buf[index].lane_id = (PRESTEER ? steer(PAY.buf[index].fu) : fu_lane_matrix[(unsigned int)PAY.buf[index].fu]);

      // FIX_ME #7
      // Dispatch the instruction into the Active List.
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. The renamer class' function for dispatching an instruction into its Active List takes nine different arguments.
      //    Six of these arguments are control flags:
      //    a. dest_valid: The instruction's payload has information about whether or not the instruction
      //       has a destination register.
      //    b. load: You can efficiently detect loads by testing the instruction's flags with the IS_LOAD() macro,
      //       as shown below. Use the local variable 'load_flag' (already declared):
      //       load_flag = IS_LOAD(PAY.buf[index].flags);
      //    c. store: You can efficiently detect stores by testing the instruction's flags with the IS_STORE() macro,
      //       as shown below. Use the local variable 'store_flag' (already declared):
      //       store_flag = IS_STORE(PAY.buf[index].flags);
      //    d. branch: You can efficiently detect branches by testing the instruction's flags with the IS_BRANCH() macro,
      //       as shown below. Use the local variable 'branch_flag' (already declared):
      //       branch_flag = IS_BRANCH(PAY.buf[index].flags);
      //    e. amo: You can efficiently detect atomic memory operations by testing the instruction's flags with the IS_AMO() macro,
      //       as shown below. Use the local variable 'amo_flag' (already declared):
      //       amo_flag = IS_AMO(PAY.buf[index].flags);
      //    f. csr: You can efficiently detect system instructions by testing the instruction's flags with the IS_CSR() macro,
      //       as shown below. Use the local variable 'csr_flag' (already declared):
      //       csr_flag = IS_CSR(PAY.buf[index].flags);
      // 3. When you dispatch the instruction into the Active List, remember to *update* the instruction's
      //    payload with its Active List index.
      load_flag = IS_LOAD(PAY.buf[index].flags);
      store_flag = IS_STORE(PAY.buf[index].flags);
      branch_flag = IS_BRANCH(PAY.buf[index].flags);
      amo_flag = IS_AMO(PAY.buf[index].flags);
      csr_flag = IS_CSR(PAY.buf[index].flags);
      PAY.buf[index].AL_index = REN->dispatch_inst(PAY.buf[index].C_valid, PAY.buf[index].C_log_reg, PAY.buf[index].C_phys_reg, load_flag, store_flag, branch_flag, amo_flag, csr_flag, PAY.buf[index].pc);


      // FIX_ME #8
      // Determine initial ready bits for the instruction's three source registers.
      // These will be used to initialize the instruction's ready bits in the Issue Queue.
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. When you implement the logic for determining readiness of the three source registers, A, B, and D,
      //    put the results of your calculations into the local variables 'A_ready', 'B_ready', and 'D_ready'.
      //    These local variables are already declared for you. You can then use these ready flags
      //    when you dispatch the instruction into one of the Issue Queues (that code is coming up shortly).
      // 3. If the instruction doesn't have a given source register, then it should be declared ready
      //    since the Issue Queue must not wait for a non-existent register. On the other hand, if the
      //    instruction does have a given source register, then you must consult the renamer module
      //    to determine whether or not the register is ready.
      A_ready = REN->is_ready(PAY.buf[index].A_phys_reg);
      if(!(PAY.buf[index].A_valid))
      {
         A_ready = true;
      }

      B_ready = REN->is_ready(PAY.buf[index].B_phys_reg);
      if(!(PAY.buf[index].B_valid))
      {
         B_ready = true;
      }
      
      D_ready = REN->is_ready(PAY.buf[index].D_phys_reg);
      if(!(PAY.buf[index].D_valid))
      {
         D_ready = true;
      }

      // FIX_ME #9
      // Clear the ready bit of the instruction's destination register.
      // This is needed to synchronize future consumers.
      //
      // (TANGENT: Alternatively, this could be done when the physical register is freed. This would
      // ensure newly-allocated physical registers are initially marked as not-ready, obviating the
      // need to clear their ready bits in the Dispatch Stage. It was less complex to implement this
      // alternative in the FabScalar library, by the way.)
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. If the instruction has a destination register, then clear its ready bit; otherwise do nothing.
      if(PAY.buf[index].C_valid)
      {
         REN->clear_ready(PAY.buf[index].C_phys_reg);
      }


      // FIX_ME #10
      // Dispatch the instruction into the Issue Queue, or circumvent the Issue Queue and immediately update status in the Active List.
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. The Decode Stage has already set the 'iq' field of the instruction's payload.
      //    It is an enumerated type with three possible values (refer to payload.h), corresponding to:
      //    * Dispatch the instruction into the Issue Queue.
      //    * Skip the Issue Queue and early-complete the instruction (NOP).
      //    * Skip the Issue Queue and early-complete the instruction, moreover, post an exception (system call).
      //    The switch statement below enumerates the three cases for you. You must implement the code for each case.

      trap_t* t;
      uint64_t funct12 = PAY.buf[index].inst.csr();

      switch (PAY.buf[index].iq) {
         case SEL_IQ:
            // FIX_ME #10a
            // Dispatch the instruction into the IQ.
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
            // 2. You only need to implement one statement: a call to the Issue Queue's dispatch function.
            //    See file issue_queue.h to determine the arguments that need to be passed in. Here is some clarification:
            //    * 'index' argument: the instruction's index into PAY.buf[]
            //    * 'branch_mask' argument: pass in the branch_mask of the instruction currently being dispatched
            //      from the DISPATCH pipeline register, i.e., DISPATCH[i].branch_mask
            //    * 'lane_id' argument: pass in the instruction's lane_id, its chosen execution lane (it was determined by the steering logic, above).
            //    * 'A_valid', 'A_ready', and 'A_tag': Valid bit, ready bit (calculated above), and physical register of first source register.
            //    * 'B_valid', 'B_ready', and 'B_tag': Valid bit, ready bit (calculated above), and physical register of second source register.
            //    * 'D_valid', 'D_ready', and 'D_tag': Valid bit, ready bit (calculated above), and physical register of third source register.
            // 3. As you can see in file pipeline.h, the IQ variable is the Issue Queue itself, NOT a pointer to it.
            IQ.dispatch(index, DISPATCH[i].branch_mask, PAY.buf[index].lane_id, PAY.buf[index].A_valid, A_ready, PAY.buf[index].A_phys_reg, 
                        PAY.buf[index].B_valid, B_ready, PAY.buf[index].B_phys_reg, PAY.buf[index].D_valid, D_ready, PAY.buf[index].D_phys_reg);


            break;


         case SEL_IQ_NONE:
            // FIX_ME #10b
            // Skip IQ.
            // Set completed bit in Active List.
            // Set exception bit in Active List, for fetch-exception-related NOPs.
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).

            // *** FIX_ME #10b (part 1): Set completed bit in Active List.
            REN->set_complete(PAY.buf[index].AL_index);

        
            // Check if this is a NOP with a fetch exception.
            // If so, throw the appropriate exception.
            if (PAY.buf[index].fetch_exception) {
               if (PAY.buf[index].fetch_exception_cause == CAUSE_MISALIGNED_FETCH)
                  t = new trap_instruction_address_misaligned(PAY.buf[index].pc);
               else if (PAY.buf[index].fetch_exception_cause == CAUSE_FAULT_FETCH)
                  t = new trap_instruction_access_fault(PAY.buf[index].pc);

               PAY.buf[index].trap = t;

               // *** FIX_ME #10b (part 2): Set exception bit in Active List.
               REN->set_exception(PAY.buf[index].AL_index);


            }
            break;

         case SEL_IQ_NONE_EXCEPTION:
            // FIX_ME #10c
            // Skip IQ.
            // Set completed bit in Active List.
            // Set exception bit in Active List.
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).

            // *** FIX_ME #10c: Set both the completed bit and the exception bit in the Active List.
            REN->set_complete(PAY.buf[index].AL_index);
            REN->set_exception(PAY.buf[index].AL_index);

 
            if (funct12 == FN12_SCALL)
               t = new trap_syscall();
            else if (funct12 == FN12_SBREAK)
               t = new trap_breakpoint();
            else
               assert(0); // Should not come here.

            PAY.buf[index].trap = t;
            break;

         default:
            assert(0);
            break;
      }


      if (IS_FP_OP(PAY.buf[index].flags)) {
#ifndef RISCV_ENABLE_FPU
         // Floating-point ISA extension is disabled: illegal instruction exception.
         REN->set_exception(PAY.buf[index].AL_index);
         PAY.buf[index].trap = new trap_illegal_instruction();
#else
         if (unlikely(!(get_state()->sr & SR_EF))) {
            // Floating-point ISA extension is enabled.
            // The pipeline cannot natively execute FP instructions, however: trap to software FP library.
            REN->set_exception(PAY.buf[index].AL_index);
            PAY.buf[index].trap = new trap_fp_disabled();
        }
#endif
      }


      // Dispatch loads and stores into the LQ/SQ and record their LQ/SQ indices.
      if (IS_MEM_OP(PAY.buf[index].flags)) {
         if (!PAY.buf[index].split_store || PAY.buf[index].upper) {
            LSU.dispatch(IS_LOAD(PAY.buf[index].flags),
                         PAY.buf[index].size,
                         PAY.buf[index].left,
                         PAY.buf[index].right,
                         PAY.buf[index].is_signed,
                         index,
                         PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase,
                         PAY.buf[index].SQ_index, PAY.buf[index].SQ_phase,
			 (IS_LOAD(PAY.buf[index].flags) && (!SPEC_DISAMBIG || (MEM_DEP_PRED && (MDP.find(PAY.buf[index].pc) != MDP.end())))));

            // The lower part of a split-store should inherit the same LSU indices.
            if (PAY.buf[index].split_store) {
               assert(PAY.buf[index+1].split && !PAY.buf[index+1].upper);
               PAY.buf[index+1].LQ_index = PAY.buf[index].LQ_index;
               PAY.buf[index+1].LQ_phase = PAY.buf[index].LQ_phase;
               PAY.buf[index+1].SQ_index = PAY.buf[index].SQ_index;
               PAY.buf[index+1].SQ_phase = PAY.buf[index].SQ_phase;
            }

            // Oracle memory disambiguation support.
            if (ORACLE_DISAMBIG && PAY.buf[index].good_instruction && IS_STORE(PAY.buf[index].flags)) {
               // Get pointer to the corresponding instruction in the functional simulator.
               actual = get_pipe()->peek(PAY.buf[index].db_index);

               // Place oracle store address into SQ before all subsequent loads are dispatched.
               // This policy ensures loads only stall on truly-dependent stores.
               LSU.store_addr(cycle, actual->a_addr, PAY.buf[index].SQ_index, PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase);
            }
         }
      }

      // Checkpointed branches must record information for restoring the LQ/SQ when a branch misprediction resolves.
      if (PAY.buf[index].checkpoint) {
         LSU.checkpoint(PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase, PAY.buf[index].SQ_index, PAY.buf[index].SQ_phase);
      }
   }

   // Remove the dispatch bundle from the Dispatch Stage.
   for (i = 0; i < dispatch_width; i++) {
      assert(DISPATCH[i].valid);
      DISPATCH[i].valid = false;
   }
}


unsigned int pipeline_t::steer(fu_type fu) {
   unsigned int fu_lane_vector;
   unsigned int lane_id;
   unsigned int i;
   bool found;

   // Choose an execution lane for the instruction based on:
   // (1) its FU type,
   // (2) the FU/lane matrix, and
   // (3) a load balancing policy.

   assert((unsigned int)fu < (unsigned int)NUMBER_FU_TYPES);
   fu_lane_vector = fu_lane_matrix[(unsigned int)fu];
   lane_id = fu_lane_ptr[(unsigned int)fu];

   assert(lane_id < issue_width);
   found = false;
   for (i = 0; i < issue_width; i++) {
      lane_id++;
      if (lane_id == issue_width) {
         lane_id = 0;
      }

      if (fu_lane_vector & (1 << lane_id)) {
         found = true;
         break;
      }
   }

   assert(found);
   fu_lane_ptr[(unsigned int)fu] = lane_id;
   return(lane_id);
}
