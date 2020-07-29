#include "pipeline.h"


void pipeline_t::writeback(unsigned int lane_number) {
   unsigned int index;

   // Check if there is an instruction in the Writeback Stage of the specified Execution Lane.
   if (Execution_Lanes[lane_number].wb.valid) {

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Get the instruction's index into PAY.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      index = Execution_Lanes[lane_number].wb.index;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // FIX_ME #15
      // Resolve branches.
      //
      // Background: Here we are resolving a branch that previously made a checkpoint.
      //
      // If the branch was correctly predicted, then resolution consists of two steps:
      // * Tell the renamer module that the branch resolved correctly, so that it frees its corresponding checkpoint.
      // * Clear the branch's bit in the branch masks of instructions in the pipeline. Specifically, this
      //   involves instructions from the Rename Stage (where branch masks are first assigned) to the
      //   Writeback Stage (instructions leave the pipeline after Writeback, although they still hold
      //   entries in the Active List and Load/Store Queues).
      //
      // If the branch was mispredicted, then resolution consists of two high-level steps:
      // * Recover key units: the Fetch Unit, the renamer module (RMT, FL, AL), and the LSU.
      // * Squash instructions in the pipline that are logically after the branch -- those instructions
      //   that have the branch's bit in their branch masks -- meanwhile clearing that bit. Also, all
      //   instructions in the frontend stages are automatically squashed since they are by definition
      //   logically after the branch.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (PAY.buf[index].checkpoint) {

         if (PERFECT_BRANCH_PRED) {
            // TODO: This assert fails due to asynchrony caused by HTIF ticks.
            // A branch may have already went in the taken direction in ISA sim
            // since a HTIF tick followed by CSR read instructions might have
            // already updated the condition check source register for this branch.
            // The same source register won't be updated until the corresponding HTIF
            // tick has happened in micro_sim and the corresponding CSR instruction has
            // retired in micro_sim. The CSR read instruction will force a recovery and
            // the next time this branch is executed, it will calculate the right value.

            //assert(PAY.buf[index].next_pc == PAY.buf[index].c_next_pc);
            assert((PAY.buf[index].next_pc == PAY.buf[index].c_next_pc) || !PAY.buf[index].good_instruction || SPEC_DISAMBIG);

            // FIX_ME #15a
            // The simulator is running in perfect branch prediction mode, therefore, all branches are correctly predicted.
            // The assertion immediately above confirms that the prediction (next_pc is the predicted target)
            // matches the outcome (c_next_pc is the calculated target).
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
            // 2. Call the resolve() function of the renamer module so that it frees the branch's checkpoint.
            //    Recall that the arguments to resolve() are:
            //    * Active List index
            //    * The branch's ID
            //    * Whether or not the branch was predicted correctly: in this case it is correct
            // 3. Do NOT worry about clearing the branch's bit in the branch masks of instructions in the pipeline.
            //    This is unnecessary since instructions don't need accurate branch masks in perfect branch prediction
            //    mode... since they are never squashed anyway.
            REN->resolve(PAY.buf[index].AL_index, PAY.buf[index].branch_ID, true);


         }
         else if (PAY.buf[index].next_pc == PAY.buf[index].c_next_pc) {
            // Branch was predicted correctly.

            // FIX_ME #15b
            // The simulator is running in real branch prediction mode, and the branch was correctly predicted.
            // You can see this in the comparison above: the prediction (next_pc is the predicted target)
            // matches the outcome (c_next_pc is the calculated target).
            //
            // Tips:
            // 1. See #15a, item 1.
            // 2. See #15a, item 2.
            // 3. Clear the branch's bit in the branch masks of instructions in the pipeline.
            //    To do this, call the resolve() function with the appropriate arguments. This function does the work for you.
            //    * resolve() is a private function of the pipeline_t class, therefore, just call it literally as 'resolve'.
            //    * resolve() takes two arguments. The first argument is the branch's ID. The second argument is a flag that
            //      indicates whether or not the branch was predicted correctly: in this case it is correct.
            //    * See pipeline.h for details about the two arguments of resolve().
            REN->resolve(PAY.buf[index].AL_index, PAY.buf[index].branch_ID, true);
            resolve(PAY.buf[index].branch_ID, true);


         }
         else {
            // Branch was mispredicted.

            // Roll-back the fetch unit: PC and branch predictor.
            pc = PAY.buf[index].c_next_pc;					// PC gets the correct target of the resolved branch.
            BP.fix_pred(PAY.buf[index].pred_tag, PAY.buf[index].c_next_pc);	// Roll-back the branch predictor to the point of the resolved branch.
 
            // Clear the fetch unit's exception, amo, and csr flags.  The fetch unit stalls on these conditions.
            // Therefore, we can infer that the mispredicted branch is logically before any offending instruction,
            // and the condition should be cleared.
            clear_fetch_exception();
            clear_fetch_amo();
            clear_fetch_csr();

            // FIX_ME #15c
            // The simulator is running in real branch prediction mode, and the branch was mispredicted.
            // Recall the two high-level steps that are necessary in this case:
            // * Recover key units: the Fetch Unit, the renamer module (RMT, FL, AL), and the LSU.
            //   The Fetch Unit is recovered in the statements immediately preceding this comment (please study them for knowledge).
            //   The LSU is recovered in the statements immediately following this comment (please study them for knowledge).
            //   Your job for #15c will be to recover the renamer module, in between.
            // * Squash instructions in the pipeline that are logically after the branch. You will do this too, in #15d below.
            //
            // Restore the RMT, FL, and AL.
            //
            // Tips:
            // 1. See #15a, item 1.
            // 2. See #15a, item 2 -- EXCEPT in this case the branch was mispredicted, so specify not-correct instead of correct.
            //    This will restore the RMT, FL, and AL, and also free this and future checkpoints... etc.
            REN->resolve(PAY.buf[index].AL_index, PAY.buf[index].branch_ID, false);


            // Restore the LQ/SQ.
            LSU.restore(PAY.buf[index].LQ_index, PAY.buf[index].LQ_phase, PAY.buf[index].SQ_index, PAY.buf[index].SQ_phase);

            // FIX_ME #15d
            // Squash instructions after the branch in program order, in all pipeline registers and the IQ.
            //
            // Tips:
            // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
            // 2. Squash instructions after the branch in program order.
            //    To do this, call the resolve() function with the appropriate arguments. This function does the work for you.
            //    * resolve() is a private function of the pipeline_t class, therefore, just call it literally as 'resolve'.
            //    * resolve() takes two arguments. The first argument is the branch's ID. The second argument is a flag that
            //      indicates whether or not the branch was predicted correctly: in this case it is not-correct.
            //    * See pipeline.h for details about the two arguments of resolve().
            resolve(PAY.buf[index].branch_ID, false);


            // Rollback PAY to the point of the branch.
            PAY.rollback(index);
         }
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // FIX_ME #16
      // Set completed bit in Active List.
      //
      // Tips:
      // 1. At this point of the code, 'index' is the instruction's index into PAY.buf[] (payload).
      // 2. Set the completed bit for this instruction in the Active List.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      REN->set_complete(PAY.buf[index].AL_index);


      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Remove the instruction from the Execution Lane.
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      Execution_Lanes[lane_number].wb.valid = false;
   }
}
