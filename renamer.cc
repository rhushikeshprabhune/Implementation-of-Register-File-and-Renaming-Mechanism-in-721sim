#include "renamer.h"
#include <inttypes.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h> 

using namespace std;

renamer::renamer(uint64_t n_log_regs,
		uint64_t n_phys_regs,
		uint64_t n_branches)
{
    physical_reg = n_phys_regs;
    logical_reg = n_log_regs;
    num_branch_unreslvd = n_branches;
    assert(physical_reg > logical_reg);
    assert(1 <= num_branch_unreslvd <= 64);
   
    /////allocate space for RMT, AMT//////
    RMT = new uint64_t[logical_reg];
    AMT = new uint64_t[logical_reg];

    //////allocate space for free list, active list, physical register file and its ready bits
    free_list.flist = new uint64_t[physical_reg - logical_reg];
    active_list.alist = new AList[physical_reg - logical_reg];
    phy_reg_file = new uint64_t[physical_reg];
    phy_reg_file_rdy_bit = new bool[physical_reg];

    //////////allocate space for checkpointS/////////????
    checkpoints = new BranchCheckpoints[num_branch_unreslvd];
    for (uint64_t i=0; i<num_branch_unreslvd; i++)
	{
        checkpoints[i].checkpointed_RMT = new uint64_t[logical_reg];
        //checkpoints[i].checkpointed_GBM = 0;
        //checkpoints[i].checkpointed_head_flist = 0;
	}

    /////////initialise checkpoints///////////////////
    ///not needed, going to write to it before read

    /////////initialise active list and free list/////////
    //head=tail=0
    free_list.head_flist = 0;
    free_list.tail_flist = 0;
    free_list.FLsize = physical_reg - logical_reg;
    for(uint64_t i=0; i< (physical_reg - logical_reg); i++)
    {
        free_list.flist[i] = i + logical_reg;
    }

    active_list.head_alist = 0;
    active_list.tail_alist = 0;
    active_list.ALsize = 0;
    for(uint64_t i=0; i< (physical_reg - logical_reg); i++)
    {
        active_list.alist[i].PC = 0;
        active_list.alist[i].physical_reg_alist = 0;
        active_list.alist[i].logical_reg_alist = 0;
        active_list.alist[i].dest_flag = 0; 
        active_list.alist[i].completed_bit = 0; 
        active_list.alist[i].exception_bit = 0; 
        active_list.alist[i].load_violation_bit = 0; 
        active_list.alist[i].branch_misprediction_bit = 0; 
        active_list.alist[i].value_misprediction_bit = 0;
	    active_list.alist[i].load_flag = 0;
        active_list.alist[i].store_flag = 0; 
        active_list.alist[i].branch_flag = 0; 
        active_list.alist[i].amo_flag = 0; 
        active_list.alist[i].csr_flag = 0;
    }

    ///////////////////initialise GBM//////////////////////////////
    GBM = 0;

    ///////////initialise RMT, AMT, PRF, PRF ready bits////////////
    for(uint64_t i=0; i < logical_reg; i++)
    {
        RMT[i] = i;
        AMT[i] = i;
    }

    for(uint64_t j = 0; j < physical_reg; j++)
    {
        phy_reg_file[j] = j;
        phy_reg_file_rdy_bit[j] = 1;
    }
}

renamer::~renamer()
{
    delete[] RMT;
    delete[] AMT;
    delete[] phy_reg_file;
    delete[] phy_reg_file_rdy_bit;
    for(uint64_t i=0; i<num_branch_unreslvd; i++)
	{
		delete[] checkpoints[i].checkpointed_RMT;
	}
    delete[] checkpoints;
    delete[] free_list.flist;
    delete[] active_list.alist;
};

void renamer::copy_AMT_to_RMT()
	{
		for(uint64_t i=0; i< logical_reg; i++)
		{
			RMT[i] = AMT[i];
		}
	}   
///////////////////Rename Stage Functions///////////////////////
bool renamer::stall_reg(uint64_t bundle_dst)
{
    uint64_t free_reg = free_list.FLsize;
    if(free_reg >= bundle_dst)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool renamer::stall_branch(uint64_t bundle_branch)
{
    uint64_t free_checkpoints = 0;
    for(uint64_t j=0; j<num_branch_unreslvd; j++)
    {
        if((GBM & (1<<j)) == 0)
        {
           free_checkpoints++;
        }
    }
    
    if(free_checkpoints >= bundle_branch)
    {
        return false;
    }
    else
    {
        return true;
    }
}

uint64_t renamer::get_branch_mask()
{
    return GBM;
}

uint64_t renamer::rename_rsrc(uint64_t log_reg)
{
    return RMT[log_reg];
}

uint64_t renamer::rename_rdst(uint64_t log_reg)
{
    assert(free_list.FLsize != 0);
    uint64_t flhead = free_list.flist[free_list.head_flist];
    free_list.head_flist++;
    if(free_list.head_flist == (physical_reg-logical_reg))
       free_list.head_flist = 0;
    
    free_list.FLsize--;
    
    phy_reg_file_rdy_bit[free_list.flist[free_list.head_flist]] = 0;
    RMT[log_reg] = flhead;
    return flhead;
}

uint64_t renamer::checkpoint()
{ 
    uint64_t branch_id;
    for(int i = 0; i < num_branch_unreslvd; i++)
    {
        uint64_t rb = ((1<<i) & GBM);
        if(rb == 0)
        {
            branch_id = i;
            break;
        }
    }
    ////////allocate a checkpoint/////// 
    assert(((1<<branch_id) & GBM) == 0);
    
    GBM = ((1<<branch_id) | GBM);
    checkpoints[branch_id].checkpointed_GBM = GBM;
    checkpoints[branch_id].checkpointed_head_flist = free_list.head_flist;
    for(uint64_t j=0; j< logical_reg; j++)
    {
        checkpoints[branch_id].checkpointed_RMT[j] = RMT[j];
    }

    return branch_id;    
}

/////////////////Dispatch Stage Functions//////////////////
bool renamer::stall_dispatch(uint64_t bundle_inst)
{
    uint64_t free_regs = physical_reg - logical_reg - active_list.ALsize;
    if(free_regs >= bundle_inst)
    {
        return false;
    }
    else
    {
        return true;
    }
}

uint64_t renamer::dispatch_inst(bool dest_valid,
	                       uint64_t log_reg,
	                       uint64_t phys_reg,
	                       bool load,
	                       bool store,
	                       bool branch,
	                       bool amo,
	                       bool csr,
	                       uint64_t PC)
                           {
                               assert(active_list.ALsize != (physical_reg - logical_reg));
                               uint64_t return_tail = active_list.tail_alist;
                               if(dest_valid == true)
                               {
                                   active_list.alist[active_list.tail_alist].logical_reg_alist = log_reg; 
                                   active_list.alist[active_list.tail_alist].physical_reg_alist = phys_reg;
                               }
                               active_list.alist[active_list.tail_alist].dest_flag = dest_valid;
                               active_list.alist[active_list.tail_alist].completed_bit = false;
                               active_list.alist[active_list.tail_alist].exception_bit = false;
                               active_list.alist[active_list.tail_alist].load_violation_bit = false;
                               active_list.alist[active_list.tail_alist].branch_misprediction_bit = false;
                               active_list.alist[active_list.tail_alist].value_misprediction_bit = false;
                               active_list.alist[active_list.tail_alist].load_flag = load;
                               active_list.alist[active_list.tail_alist].store_flag = store;
                               active_list.alist[active_list.tail_alist].branch_flag = branch;
                               active_list.alist[active_list.tail_alist].amo_flag = amo;
                               active_list.alist[active_list.tail_alist].csr_flag = csr;
                               active_list.alist[active_list.tail_alist].PC = PC;

                               active_list.tail_alist++; 
                               if(active_list.tail_alist == (physical_reg - logical_reg))
                                  active_list.tail_alist = 0;
                               
                               active_list.ALsize++;
                               return return_tail;                       
                           }

//////////////Schedule Stage Functions//////////////////////
bool renamer::is_ready(uint64_t phys_reg)
{
    return (phy_reg_file_rdy_bit[phys_reg] == 1);
}

void renamer::clear_ready(uint64_t phys_reg)
{
    phy_reg_file_rdy_bit[phys_reg] = 0;
}

void renamer::set_ready(uint64_t phys_reg)
{
    phy_reg_file_rdy_bit[phys_reg] = 1;
}

/////////////Register Read Stage///////////////////////////
uint64_t renamer::read(uint64_t phys_reg)
{
    uint64_t value;
    value = phy_reg_file[phys_reg];
    return value;
}

/////////////Writeback Stage Functions////////////////////
void renamer::write(uint64_t phys_reg, uint64_t value)
{
    phy_reg_file[phys_reg] = value;
}

void renamer::set_complete(uint64_t AL_index)
{
    active_list.alist[AL_index].completed_bit = 1;
}

void renamer::resolve(uint64_t AL_index,
		     uint64_t branch_ID,
		     bool correct)
             {
                if(correct == true)
                {
                   GBM = GBM & ~(1<<branch_ID);
                   for (uint64_t i=0; i<num_branch_unreslvd; i++)
	               {
                     checkpoints[i].checkpointed_GBM = checkpoints[i].checkpointed_GBM & (~(1<<branch_ID));
	               }
                }
                else if(correct == false)
                {   
                    GBM = checkpoints[branch_ID].checkpointed_GBM;
                    assert((GBM & (1<<branch_ID)) > 0);
                    GBM = GBM & ~(1<<branch_ID);

                    AL_index++;
                    if(AL_index == (physical_reg - logical_reg))
                       AL_index = 0;

                    while(active_list.tail_alist != AL_index)
                    {
                        if(active_list.tail_alist == 0)
                        {
                            active_list.tail_alist = physical_reg - logical_reg - 1;
                        }
                        else
                        {
                            active_list.tail_alist--;
                        }
                        active_list.ALsize--;
                    }
                    assert(active_list.tail_alist == AL_index);
                    
                   
                    ////restore free list
                    while(free_list.head_flist != checkpoints[branch_ID].checkpointed_head_flist)
                    {
                        //phy_reg_file_rdy_bit[free_list.flist[free_list.head_flist]] = 1;
                        if(free_list.head_flist == 0)
                        {
                            free_list.head_flist = physical_reg - logical_reg - 1;
                        }
                        else
                        {
                            free_list.head_flist--;
                        }
                        free_list.FLsize++;
                    }
                    uint64_t i = free_list.FLsize;
                    uint64_t j = free_list.head_flist;
                    while(i>0)
                    {
                        phy_reg_file_rdy_bit[free_list.flist[j]] = 1;
                        j++;
                        if(j == physical_reg - logical_reg)
                        {
                            j = 0;
                        }
                        i--;
                    }
                    assert(free_list.head_flist == checkpoints[branch_ID].checkpointed_head_flist);
                    for(uint64_t i=0; i<logical_reg; i++)
                    { 
                        RMT[i] = checkpoints[branch_ID].checkpointed_RMT[i];
                    }
                }
             }

///////////Retire Stage Functions//////////////////////
bool renamer::precommit(bool &completed,
                       bool &exception, bool &load_viol, bool &br_misp, bool &val_misp,
	               bool &load, bool &store, bool &branch, bool &amo, bool &csr,
		       uint64_t &PC)
               {
                //    completed = active_list.alist[active_list.head_alist].completed_bit;
                //    exception = active_list.alist[active_list.head_alist].exception_bit;
                //    load_viol = active_list.alist[active_list.head_alist].load_violation_bit;
                //    br_misp = active_list.alist[active_list.head_alist].branch_misprediction_bit;
                //    val_misp = active_list.alist[active_list.head_alist].value_misprediction_bit;
                //    load = active_list.alist[active_list.head_alist].load_flag;
                //    store = active_list.alist[active_list.head_alist].store_flag;
                //    branch = active_list.alist[active_list.head_alist].branch_flag;
                //    amo = active_list.alist[active_list.head_alist].amo_flag;
                //    csr = active_list.alist[active_list.head_alist].csr_flag;
                //    PC = active_list.alist[active_list.head_alist].PC;
                   if(active_list.ALsize == 0)
                   return false;
                   else
                   {
                        completed = active_list.alist[active_list.head_alist].completed_bit;
                   exception = active_list.alist[active_list.head_alist].exception_bit;
                   load_viol = active_list.alist[active_list.head_alist].load_violation_bit;
                   br_misp = active_list.alist[active_list.head_alist].branch_misprediction_bit;
                   val_misp = active_list.alist[active_list.head_alist].value_misprediction_bit;
                   load = active_list.alist[active_list.head_alist].load_flag;
                   store = active_list.alist[active_list.head_alist].store_flag;
                   branch = active_list.alist[active_list.head_alist].branch_flag;
                   amo = active_list.alist[active_list.head_alist].amo_flag;
                   csr = active_list.alist[active_list.head_alist].csr_flag;
                   PC = active_list.alist[active_list.head_alist].PC;
                   return true;
                   }
                
               }

void renamer::commit()
{
    assert(active_list.ALsize != 0);
    assert(active_list.alist[active_list.head_alist].completed_bit == 1);
    assert(active_list.alist[active_list.head_alist].exception_bit == 0);
    assert(active_list.alist[active_list.head_alist].load_violation_bit == 0);
    assert(active_list.alist[active_list.head_alist].branch_misprediction_bit == 0);

    /////free phy reg in amt to free list and put current mapping of logical reg in active list to AMT///////DONE
    if(active_list.alist[active_list.head_alist].dest_flag == 1)
    {
        free_list.flist[free_list.tail_flist] = AMT[active_list.alist[active_list.head_alist].logical_reg_alist];
        free_list.tail_flist++;
        if(free_list.tail_flist == (physical_reg - logical_reg))  
           free_list.tail_flist = 0;
        
        free_list.FLsize++;
        AMT[active_list.alist[active_list.head_alist].logical_reg_alist] = active_list.alist[active_list.head_alist].physical_reg_alist;
    }
    
     active_list.head_alist++;
    if(active_list.head_alist == (physical_reg - logical_reg))
       active_list.head_alist = 0;
    
    active_list.ALsize--;
}

///////////Squash Function//////////////////
void renamer::squash()
{
    copy_AMT_to_RMT();
    free_list.head_flist = free_list.tail_flist;
    free_list.FLsize = physical_reg - logical_reg;
    uint64_t i = free_list.FLsize;
    uint64_t j = free_list.head_flist;
    while(i!=0)
        {
            phy_reg_file_rdy_bit[free_list.flist[j]] = 1;
            j++;
            if(j == physical_reg - logical_reg)
            {
                j = 0;
            }
            i--;
        }
    active_list.tail_alist = active_list.head_alist = 0;
    active_list.ALsize = 0;
    GBM=0;
}

//////Random required Functions////////////
void renamer::set_exception(uint64_t AL_index)
{
    active_list.alist[AL_index].exception_bit = 1;
}

void renamer::set_load_violation(uint64_t AL_index)
{
    active_list.alist[AL_index].load_violation_bit = 1;
}

void renamer::set_branch_misprediction(uint64_t AL_index)
{
    active_list.alist[AL_index].branch_misprediction_bit = 1;
}

void renamer::set_value_misprediction(uint64_t AL_index)
{
    active_list.alist[AL_index].value_misprediction_bit = 1;
}
	
bool renamer::get_exception(uint64_t AL_index)
{
    return(active_list.alist[AL_index].exception_bit);
}
