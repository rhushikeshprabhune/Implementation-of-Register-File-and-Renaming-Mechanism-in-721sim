#include "pipeline.h"
#include "trap.h"


void pipeline_t::execute(unsigned int lane_number) {
   unsigned int index;
   bool hit;		// load value available
   unsigned int depth;

   // Check if there is an instruction in the final Execute Stage of the specified Execution Lane.
   assert(Execution_Lanes[lane_number].ex_depth > 0);
   depth = (Execution_Lanes[lane_number].ex_depth - 1);
   if (Execution_Lanes[lane_number].ex[depth].valid) {

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Get the instruction's index into PAY.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      index = Execution_Lanes[lane_number].ex[depth].index;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Execute the instruction.
      // * Load and store instructions use the AGEN and Load/Store Units.
      // * All other instructions use the ALU.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (IS_MEM_OP(PAY.buf[index].flags)) {
         // Perform AGEN to generate address.
         if (!PAY.buf[index].split_store || PAY.buf[index].upper) {
            agen(index);
         }

         // Execute the load or store in the LSU.
 
         if (IS_LOAD(PAY.buf[index].flags)) {
            // Instruction is a load.

            if (IS_AMO(PAY.buf[index].flags)) {
               // Set up load reservation to verifiy atomicity with a following store-conditional.
               get_state()->load_reservation = PAY.buf[index].addr;
            }

            hit = LSU.load_addr(cycle,
                                PAY.buf[index].addr,
                                PAY.buf[index].LQ_index,
                                PAY.buf[index].SQ_index, PAY.buf[index].SQ_phase,
                                PAY.buf[index].C_value.dw);

            // FIX_ME #13
            // If the load hit:
            // (1) Broadcast its destination tag to the IQ to wakeup its dependent instructions.
            // (2) Set the corresponding ready bit in the Physical Register File Ready Bit Array.
            // (3) Write the value into the Physical Register File. Doing this here, instead of in WB,
            //     properly simulates the bypass network.
            //
            // If it didn't hit, it will get replayed later from within the LSU ("load_replay").
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
            // 2. Background: The code above attempts to execute the load instruction in the LSU.
            //    The load may hit (a value is obtained from either the SQ or D$) or not hit (disambiguation stall or D$ miss stall).
            //    The local variable 'hit' indicates which case occurred. Recall, since we didn't know in the Register Read Stage
            //    whether or not the load would hit, we didn't speculatively wakeup dependent instructions in that Stage. Now we know
            //    if it hit or not. If it did hit, we need to not only write the load value into the Physical Register File, but also
            //    wakeup dependent instructions and set the destination register's ready bit -- two tasks that were deferred until we
            //    knew for certain that we could.
            // 3. ONLY if the load instruction has hit (check local variable 'hit', already declared and set above):
            //    a. Wakeup dependents in the IQ using its wakeup() port (see issue_queue.h for arguments to the wakeup port).
            //    b. Set the destination register's ready bit.
            //    c. Write the doubleword value of the destination register (now available in the instruction's payload, which was
            //       provided by the LSU via the code above) into the Physical Register File.
            //       Note: Values in the payload use a union type (can be referenced as either a single doubleword or as two words
            //       separately); see the comments in file payload.h regarding referencing a value as a single doubleword.
            if(hit)
            {
               IQ.wakeup(PAY.buf[index].C_phys_reg);
               REN->set_ready(PAY.buf[index].C_phys_reg);
               REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);
            }


         }
         else {
            // Instruction is a store
            assert(IS_STORE(PAY.buf[index].flags));

            if (PAY.buf[index].split_store) {
               assert(PAY.buf[index].split);
               if (PAY.buf[index].upper)
                  LSU.store_addr(cycle, PAY.buf[index].addr, PAY.buf[index].SQ_index, PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase);    // upper op: address
               else
                  LSU.store_value(PAY.buf[index].SQ_index, PAY.buf[index].A_value.dw);    // lower op: value
            }
            else {
               // If not a split-store, then the store has both the address and the value.
               LSU.store_addr(cycle, PAY.buf[index].addr, PAY.buf[index].SQ_index, PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase);
               LSU.store_value(PAY.buf[index].SQ_index, PAY.buf[index].B_value.dw);
            }

            // Store-conditional: write 0 to its destination register anticipating a success.
            if (IS_AMO(PAY.buf[index].flags)) {
               assert(PAY.buf[index].C_valid);
               assert(PAY.buf[index].C_log_reg != 0);  // if X0, would have cleared C_valid in Decode Stage
               PAY.buf[index].C_value.dw = 0;
               REN->write(PAY.buf[index].C_phys_reg, 0);
            }
         }
      }
      else {
         // Execute the ALU-type instruction on the ALU.
         try {
            alu(index);
         }
         // Catch exceptions thrown by the ALU.
         catch (trap_t *t) {
            unsigned int al_index = PAY.buf[index].AL_index;
            reg_t epc = PAY.buf[index].pc;
            ifprintf(logging_on,execute_log, "Cycle %" PRIcycle ": core %3d: exception %s, epc 0x%016" PRIx64 " al_index %u\n", cycle, id, t->name(), epc, al_index);
            REN->set_exception(al_index);
            PAY.buf[index].trap = t;
         }
         // Catch reference types thrown from unknown source outside micro sim.
         catch (trap_t& t){
            unsigned int al_index = PAY.buf[index].AL_index;
            reg_t epc = PAY.buf[index].pc;
            ifprintf(logging_on,execute_log, "Cycle %" PRIcycle ": core %3d: exception refernce thrown from unknown source %s, epc 0x%016" PRIx64 " al_index %u\n", cycle, id, t.name(), epc, al_index);
            trap_t *tp;
            switch(t.cause()){
               case CAUSE_FP_DISABLED:
                  tp = new trap_fp_disabled();
                  break;
               case CAUSE_ILLEGAL_INSTRUCTION:
                  tp = new trap_illegal_instruction();
                  break;
               case CAUSE_PRIVILEGED_INSTRUCTION:
                  tp = new trap_privileged_instruction();
                  break;
               default:
                  fflush(0);
                  assert(0);
                  break;
            }
            REN->set_exception(al_index);
            PAY.buf[index].trap = tp;
         }

         // FIX_ME #14
         // If the ALU type instruction has a destination register (not a branch),
         // then write its result value into the Physical Register File.
         // Doing this here, instead of in WB, properly simulates the bypass network.
         //
         // Tips:
         // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
         // 2. If the ALU type instruction has a destination register, then write the doubleword value of the
         //    destination register (now available in the instruction's payload, which was provided by alu()
         //    via the code above) into the Physical Register File.
         //    Note: Values in the payload use a union type (can be referenced as either a single doubleword or as two words
         //    separately); see the comments in file payload.h regarding referencing a value as a single doubleword.
         if(PAY.buf[index].C_valid)
         {
            REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);
         }


      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////
      // NOTE: Setting the completed bit in the Active List is deferred to the Writeback Stage.
      // NOTE: Resolving branches is deferred to the Writeback Stage.
      ////////////////////////////////////////////////////////////////////////////////////////////////////

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Advance the instruction to the Writeback Stage.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////

      // There must be space in the Writeback Stage because Execution Lanes are free-flowing.
      assert(!Execution_Lanes[lane_number].wb.valid);

      // Copy instruction to Writeback Stage.
      // BUT: Stalled loads should not advance to the Writeback Stage.
      if (!IS_LOAD(PAY.buf[index].flags) || hit) {
         Execution_Lanes[lane_number].wb.valid = true;
         Execution_Lanes[lane_number].wb.index = Execution_Lanes[lane_number].ex[depth].index;
         Execution_Lanes[lane_number].wb.branch_mask = Execution_Lanes[lane_number].ex[depth].branch_mask;
      }

      // Remove instruction from Execute Stage.
      Execution_Lanes[lane_number].ex[depth].valid = false;
   }

   if (depth > 0) {
      // This is a multi-cycle execution lane.
      // "depth" corresponds to the instruction in the last sub-stage (and was handled above).
      // "depth-1" corresponds to instruction in second-to-last sub-stage.
      
      if (Execution_Lanes[lane_number].ex[depth-1].valid) {
	 index = Execution_Lanes[lane_number].ex[depth-1].index;
         // FIX_ME #11b
         //
         // The check, above, indicates that there is a valid instruction in the
         // second-to-last sub-stage, which is the equivalent of the Register Read stage
	 // in the case of a single-cycle producer -- in terms of wakeup timing.
         //
         // If the "depth-1" instruction has a destination register AND it is not a load:
         // (1) Broadcast its destination tag to the IQ to wakeup its dependent instructions.
         // (2) Set the corresponding ready bit in the Physical Register File Ready Bit Array.
	 //
         // Tips:
         // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
         // 2. The easiest way to tell if this instruction is a load or not, is to test the instruction's
         //    flags (in its payload) via the IS_LOAD() macro (see pipeline.h).
         // 3. If the instruction has a destination register AND it is not a load, then:
         //    a. Wakeup dependents in the IQ using its wakeup() port (see issue_queue.h for arguments
         //       to the wakeup port).
         //    b. Set the destination register's ready bit.
         if((!IS_LOAD(PAY.buf[index].flags)) && (PAY.buf[index].C_valid))
      {
         IQ.wakeup(PAY.buf[index].C_phys_reg);
         REN->set_ready(PAY.buf[index].C_phys_reg);
      }
      }
   }

   // Advance instructions that are in-flight within the Execution Lane.
   while (depth > 0) {
      if (Execution_Lanes[lane_number].ex[depth-1].valid) {
         // There must be space in the next sub-stage.
         assert(!Execution_Lanes[lane_number].ex[depth].valid);

         // Copy instruction from [depth-1] sub-stage to [depth] sub-stage.
         Execution_Lanes[lane_number].ex[depth].valid = true;
         Execution_Lanes[lane_number].ex[depth].index = Execution_Lanes[lane_number].ex[depth-1].index;
         Execution_Lanes[lane_number].ex[depth].branch_mask = Execution_Lanes[lane_number].ex[depth-1].branch_mask;

         // Remove instruction from [depth-1] sub-stage.
         Execution_Lanes[lane_number].ex[depth-1].valid = false;
      }
      depth--;
   }
}

void pipeline_t::load_replay() {
   //////////////////////////////
   // FIX_ME #18
   // Replay stalled loads.
   //
   // There is an autonomous engine that replays one stalled load each cycle in the LSU,
   // to determine if it can unstall. The code, below, implements the autonomous replay engine.
   // If the replay succeeds, it means the load finally has a value and we can
   // (1) wakeup its dependents,
   // (2) set the ready bit of its destination register, and
   // (3) write its value into the Physical Register File.
   // We already did these three steps for loads that hit on the first attempt;
   // here we are doing them for loads that stalled and have finally become unstalled.
   //////////////////////////////

   unsigned int index;
   reg_t value;
   if (LSU.load_unstall(cycle, index, value)) {
      // Load has resolved.
      assert(IS_LOAD(PAY.buf[index].flags));
      assert(PAY.buf[index].C_valid);
      PAY.buf[index].C_value.dw = value;

      // FIX_ME #18a
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. See #13 (in execute.cc), and implement steps 3a,3b,3c.
      IQ.wakeup(PAY.buf[index].C_phys_reg);
      REN->set_ready(PAY.buf[index].C_phys_reg);
      REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);


      // FIX_ME #18b
      // Set completed bit in Active List.
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. Set the completed bit for this instruction in the Active List.
      REN->set_complete(PAY.buf[index].AL_index);


   }
}
