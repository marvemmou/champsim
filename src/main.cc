/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <getopt.h>
#include <iomanip>
#include <signal.h>
#include <string.h>
#include <vector>

#include "cache.h"
#include "champsim.h"
#include "champsim_constants.h"
#include "dram_controller.h"
//#include "dramsim3_wrapper.hpp"
#include "ooo_cpu.h"
#include "operable.h"
#include "tracereader.h"
#include "vmem.h"

uint8_t warmup_complete[NUM_CPUS] = {}, simulation_complete[NUM_CPUS] = {}, smt_warmup_complete[8] = {}, smt_simulation_complete[8] = {}, all_warmup_complete = 0, all_simulation_complete = 0, smt_all_warmup_complete[NUM_CPUS] = {}, smt_all_simulation_complete[NUM_CPUS] = {},
        MAX_INSTR_DESTINATIONS = NUM_INSTR_DESTINATIONS, knob_cloudsuite = 0, knob_low_bandwidth = 0;

uint64_t warmup_instructions, simulation_instructions;

uint64_t switch_policy = 0;

typedef enum{
  RR = 0,
  ICOUNT = 1,
  ON_DETECT_MISS = 2,
  ON_PREDICT_MISS = 3,
} policies;

auto start_time = time(NULL);

// For backwards compatibility with older module source.
champsim::deprecated_clock_cycle current_core_cycle;

extern MEMORY_CONTROLLER DRAM;
//extern DRAMSim3_DRAM DRAM;
extern VirtualMemory vmem;
extern std::array<O3_CPU*, NUM_CPUS> ooo_cpu;
extern std::array<CACHE*, NUM_CACHES> caches;
extern std::array<champsim::operable*, NUM_OPERABLES> operables;

std::vector<tracereader*> traces;

uint64_t champsim::deprecated_clock_cycle::operator[](std::size_t cpu_idx)
{
  static bool deprecate_printed = false;
  if (!deprecate_printed) {
    std::cout << "WARNING: The use of 'current_core_cycle[cpu]' is deprecated." << std::endl;
    std::cout << "WARNING: Use 'this->current_cycle' instead." << std::endl;
    deprecate_printed = true;
  }
  return ooo_cpu[cpu_idx]->current_cycle;
}

void record_roi_stats(uint32_t cpu, CACHE* cache)
{
  for (uint32_t i = 0; i < NUM_TYPES; i++) {
    cache->roi_access[cpu][i] = cache->sim_access[cpu][i];
    cache->roi_hit[cpu][i] = cache->sim_hit[cpu][i];
    cache->roi_miss[cpu][i] = cache->sim_miss[cpu][i];
  }
}

void print_roi_stats(uint32_t cpu, CACHE* cache)
{
  uint64_t TOTAL_ACCESS = 0, TOTAL_HIT = 0, TOTAL_MISS = 0;

  for (uint32_t i = 0; i < NUM_TYPES; i++) {
    TOTAL_ACCESS += cache->roi_access[cpu][i];
    TOTAL_HIT += cache->roi_hit[cpu][i];
    TOTAL_MISS += cache->roi_miss[cpu][i];
  }

  if (TOTAL_ACCESS > 0) {
    cout << cache->NAME;
    cout << " TOTAL     ACCESS: " << setw(10) << TOTAL_ACCESS << "  HIT: " << setw(10) << TOTAL_HIT << "  MISS: " << setw(10) << TOTAL_MISS << endl;

    cout << cache->NAME;
    cout << " LOAD      ACCESS: " << setw(10) << cache->roi_access[cpu][0] << "  HIT: " << setw(10) << cache->roi_hit[cpu][0] << "  MISS: " << setw(10)
         << cache->roi_miss[cpu][0] << endl;

    cout << cache->NAME;
    cout << " RFO       ACCESS: " << setw(10) << cache->roi_access[cpu][1] << "  HIT: " << setw(10) << cache->roi_hit[cpu][1] << "  MISS: " << setw(10)
         << cache->roi_miss[cpu][1] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  ACCESS: " << setw(10) << cache->roi_access[cpu][2] << "  HIT: " << setw(10) << cache->roi_hit[cpu][2] << "  MISS: " << setw(10)
         << cache->roi_miss[cpu][2] << endl;

    cout << cache->NAME;
    cout << " WRITEBACK ACCESS: " << setw(10) << cache->roi_access[cpu][3] << "  HIT: " << setw(10) << cache->roi_hit[cpu][3] << "  MISS: " << setw(10)
         << cache->roi_miss[cpu][3] << endl;

    cout << cache->NAME;
    cout << " TRANSLATION ACCESS: " << setw(10) << cache->roi_access[cpu][4] << "  HIT: " << setw(10) << cache->roi_hit[cpu][4] << "  MISS: " << setw(10)
         << cache->roi_miss[cpu][4] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  REQUESTED: " << setw(10) << cache->pf_requested << "  ISSUED: " << setw(10) << cache->pf_issued;
    cout << "  USEFUL: " << setw(10) << cache->pf_useful << "  USELESS: " << setw(10) << cache->pf_useless << endl;

    cout << cache->NAME;
    cout << " AVERAGE MISS LATENCY: " << (1.0 * (cache->total_miss_latency)) / TOTAL_MISS << " cycles" << endl;
    // cout << " AVERAGE MISS LATENCY: " <<
    // (cache->total_miss_latency)/TOTAL_MISS << " cycles " <<
    // cache->total_miss_latency << "/" << TOTAL_MISS<< endl;
  }
}

void print_cumulative_stats(uint32_t cpu) {
  // load-induced bubble stats

  for (int i=0; i<traces.size(); i++) {
      std::cout << "SMT_thread_" << i << "_bubble_called " << ooo_cpu[cpu]->stats.bubble[i].called << endl
        << "SMT_thread_" << i << "_bubble_rob_non_head " << ooo_cpu[cpu]->stats.bubble[i].rob_non_head << endl
        << "SMT_thread_" << i << "_bubble_rob_head " << ooo_cpu[cpu]->stats.bubble[i].rob_head << endl
        << "SMT_thread_" << i << "_bubble_went_offchip " << ooo_cpu[cpu]->stats.bubble[i].went_offchip << endl
        << "SMT_thread_" << i << "_bubble_went_offchip_rob_head " << ooo_cpu[cpu]->stats.bubble[i].went_offchip_rob_head << endl
        << "SMT_thread_" << i << "_bubble_went_offchip_rob_non_head " << ooo_cpu[cpu]->stats.bubble[i].went_offchip_rob_non_head << endl
        << endl;
  
        int index = 0;
        std::cout << "SMT_thread_" << i << "_bubble_rob_part_" << index << "_max " << ooo_cpu[cpu]->bubble_max[i] << endl
              << "SMT_thread_" << i << "_bubble_rob_part_" << index << "_min " << ooo_cpu[cpu]->bubble_min[i] << endl
              << "SMT_thread_" << i << "_bubble_rob_part_" << index << "_cnt " << ooo_cpu[cpu]->bubble_cnt[i] << endl
              << "SMT_thread_" << i << "_bubble_rob_part_" << index << "_tot " << ooo_cpu[cpu]->bubble_tot[i] << endl
              << "SMT_thread_" << i << "_bubble_rob_part_" << index << "_avg " << (float)ooo_cpu[cpu]->bubble_tot[i]/ooo_cpu[cpu]->bubble_cnt[i] << endl;

        std::cout << endl;
  }


  // loads per ip stats
  std::vector<std::pair<uint64_t, load_per_ip_info_t>> pairs;
  for(auto it = ooo_cpu[cpu]->load_per_ip_stats.begin(); it != ooo_cpu[cpu]->load_per_ip_stats.end(); ++it)
      pairs.push_back(*it);
  std::sort(pairs.begin(), pairs.end(), 
      [](const std::pair<uint64_t, load_per_ip_info_t> &p1, const std::pair<uint64_t, load_per_ip_info_t> &p2)
      {
          return p1.second.loads_went_offchip > p2.second.loads_went_offchip;
      });
  
  uint64_t total_loads = 0, total_loads_went_offchip = 0, accum = 0;
  for(auto it = pairs.begin(); it != pairs.end(); ++it)
  {
      total_loads += it->second.total_loads;
      total_loads_went_offchip += it->second.loads_went_offchip;
  }

  std::cout << "Core_" << cpu << "_total_loads " << total_loads << endl
         << "Core_" << cpu << "_total_loads_went_offchip " << total_loads_went_offchip << endl
         << "Core_" << cpu << "_total_frontal_loads " << ooo_cpu[cpu]->load_per_rob_part_stats.total_loads << endl
         << "Core_" << cpu << "_total_frontal_loads_went_offchip " << ooo_cpu[cpu]->load_per_rob_part_stats.loads_went_offchip << endl
         << endl;

    // print stats for top-90% off-chip load-generating PCs
    std::cout << "[Histogram of top-90\% off-chip load-generating PCs]" << endl;
    std::cout << "pc|total_loads|loads_went_offchip|loads_went_offchip_pos[0]|loads_went_offchip_pos[1]|..." << endl;
    uint64_t load_ips = 0, frontal_bias_load_ips[5] = {0};
    float bias_values[5] = {0.9, 0.8, 0.7, 0.6, 0.5};
    uint64_t frontal_bias_load_ips_total_loads[5] = {0}, 
             frontal_bias_load_ips_off_chip_loads[5] = {0}; 

    for(auto it = pairs.begin(); it != pairs.end(); ++it)
    {
        std::cout << setw(10) << hex << it->first << dec << "|"
             << setw(10) << it->second.total_loads << "|"
             << setw(10) << it->second.loads_went_offchip << "|"
            ;
        
        cout << endl;
        load_ips++;
        
        // count frontal and dorsal offchip loads
        uint64_t frontal_loads_went_offchip = it->second.loads_went_offchip_pos_hist;
        
        for(uint32_t i = 0; i < 5; ++i)
        {
            // check frontal bias
            if((float)frontal_loads_went_offchip/it->second.loads_went_offchip >= bias_values[i])
            {
                frontal_bias_load_ips[i]++;
                // keep track of total number of loads and off-chip loads generated by all frontal-biased IPs
                frontal_bias_load_ips_total_loads[i] += it->second.total_loads;
                frontal_bias_load_ips_off_chip_loads[i] += it->second.loads_went_offchip;
            }
        }
        
        accum += it->second.loads_went_offchip;
        if((float)accum/total_loads_went_offchip >= 0.9)
        {
            break;
        }
    }
    cout << endl;

    cout << "Core_" << cpu << "_total_load_ips_went_offchip " << load_ips << endl;
    for(uint32_t i = 0; i < 5; ++i)
    {
          cout   << "Core_" << cpu << "_load_ips_frontal_biased_" << bias_values[i] << " " << frontal_bias_load_ips[i] << endl;
          cout   << "Core_" << cpu << "_load_ips_frontal_biased_" << bias_values[i] << "_total_loads " << frontal_bias_load_ips_total_loads[i] << endl;

    }
    cout << endl;

    // frontal loads per ip stats
    std::vector<std::pair<uint64_t, load_per_ip_info_t>> pairs2;
    for(auto it = ooo_cpu[cpu]->frontal_load_per_ip_stats.begin(); it != ooo_cpu[cpu]->frontal_load_per_ip_stats.end(); ++it)
        pairs2.push_back(*it);
    std::sort(pairs2.begin(), pairs2.end(), 
        [](const std::pair<uint64_t, load_per_ip_info_t> &p1, const std::pair<uint64_t, load_per_ip_info_t> &p2)
        {
            return p1.second.total_loads > p2.second.total_loads;
        });
    uint64_t total_frontal_loads = 0;
    for(auto it = pairs2.begin(); it != pairs2.end(); ++it)
    {
        total_frontal_loads += it->second.total_loads;
    }
    accum = 0;
    uint64_t frontal_load_generating_ips = 0, frontal_load_generating_ips_offchip_biased[5] = {0};
    for(auto it = pairs2.begin(); it != pairs2.end(); ++it)
    {
        frontal_load_generating_ips++;
        for(uint32_t i = 0; i < 5; ++i)
        {
            if((float)it->second.loads_went_offchip/it->second.total_loads >= bias_values[i])
            {
                frontal_load_generating_ips_offchip_biased[i]++;
            }
        }

        accum += it->second.total_loads;
        if((float)accum/total_frontal_loads >= 0.9)
        {
            break;
        }
    }
    cout << "Core_" << cpu << "_total_frontal_load_generating_ips " << frontal_load_generating_ips << endl;
    for(uint32_t i = 0; i < 5; ++i)
    {
        cout << "Core_" << cpu << "_frontal_load_generating_ips_offchip_biased_" << bias_values[i] << " " << frontal_load_generating_ips_offchip_biased[i] << endl;
    }
    cout << endl;

    cout << "Core_" << cpu << "_mlp stats" << endl;
    cout << ooo_cpu[cpu]->glob_mlp_amount.desc() << endl;
    for (uint32_t i = 0; i < ooo_cpu[cpu]->glob_mlp_amount.size(); i++) {
        cout << " ";
        if (ooo_cpu[cpu]->glob_mlp_amount.hasCounterNames()) {
            cout << ooo_cpu[cpu]->glob_mlp_amount.counterName(i) << ": " << ooo_cpu[cpu]->glob_mlp_amount.count(i) << endl;
        } else {
            cout << i << ": " << ooo_cpu[cpu]->glob_mlp_amount.count(i) << endl;
        }
    }

    for(int j=0; j< traces.size(); j++) {
      cout << ooo_cpu[cpu]->mlp_amount[j].desc() << endl;
      for (uint32_t i = 0; i < ooo_cpu[cpu]->mlp_amount[j].size(); i++) {
          cout << " ";
          if (ooo_cpu[cpu]->mlp_amount[j].hasCounterNames()) {
              cout << ooo_cpu[cpu]->mlp_amount[j].counterName(i) << ": " << ooo_cpu[cpu]->mlp_amount[j].count(i) << endl;
          } else {
              cout << i << ": " << ooo_cpu[cpu]->mlp_amount[j].count(i) << endl;
          }
      }
    }



    cout << ooo_cpu[cpu]->min_mlp_dist.desc() << endl;
    for (uint32_t i = 0; i < ooo_cpu[cpu]->min_mlp_dist.size(); i++) {
        cout << " ";
        if (ooo_cpu[cpu]->min_mlp_dist.hasCounterNames()) {
            cout << ooo_cpu[cpu]->min_mlp_dist.counterName(i) << ": " << ooo_cpu[cpu]->min_mlp_dist.count(i) << endl;
        } else {
            cout << i << ": " << ooo_cpu[cpu]->min_mlp_dist.count(i) << endl;
        }
    }

    cout << ooo_cpu[cpu]->max_mlp_dist.desc() << endl;
    for (uint32_t i = 0; i < ooo_cpu[cpu]->max_mlp_dist.size(); i++) {
        cout << " ";
        if (ooo_cpu[cpu]->max_mlp_dist.hasCounterNames()) {
            cout << ooo_cpu[cpu]->max_mlp_dist.counterName(i) << ": " << ooo_cpu[cpu]->max_mlp_dist.count(i) << endl;
        } else {
            cout << i << ": " << ooo_cpu[cpu]->max_mlp_dist.count(i) << endl;
        }
    }


}

void print_sim_stats(uint32_t cpu, CACHE* cache)
{
  uint64_t TOTAL_ACCESS = 0, TOTAL_HIT = 0, TOTAL_MISS = 0;

  for (uint32_t i = 0; i < NUM_TYPES; i++) {
    TOTAL_ACCESS += cache->sim_access[cpu][i];
    TOTAL_HIT += cache->sim_hit[cpu][i];
    TOTAL_MISS += cache->sim_miss[cpu][i];
  }

  if (TOTAL_ACCESS > 0) {
    cout << cache->NAME;
    cout << " TOTAL     ACCESS: " << setw(10) << TOTAL_ACCESS << "  HIT: " << setw(10) << TOTAL_HIT << "  MISS: " << setw(10) << TOTAL_MISS << endl;

    cout << cache->NAME;
    cout << " LOAD      ACCESS: " << setw(10) << cache->sim_access[cpu][0] << "  HIT: " << setw(10) << cache->sim_hit[cpu][0] << "  MISS: " << setw(10)
         << cache->sim_miss[cpu][0] << endl;

    cout << cache->NAME;
    cout << " RFO       ACCESS: " << setw(10) << cache->sim_access[cpu][1] << "  HIT: " << setw(10) << cache->sim_hit[cpu][1] << "  MISS: " << setw(10)
         << cache->sim_miss[cpu][1] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  ACCESS: " << setw(10) << cache->sim_access[cpu][2] << "  HIT: " << setw(10) << cache->sim_hit[cpu][2] << "  MISS: " << setw(10)
         << cache->sim_miss[cpu][2] << endl;

    cout << cache->NAME;
    cout << " WRITEBACK ACCESS: " << setw(10) << cache->sim_access[cpu][3] << "  HIT: " << setw(10) << cache->sim_hit[cpu][3] << "  MISS: " << setw(10)
         << cache->sim_miss[cpu][3] << endl;
  }
}

void print_branch_stats()
{
  for (uint32_t i = 0; i < NUM_CPUS; i++) {
    cout << endl << "CPU " << i << " Branch Prediction Accuracy: ";
    cout << (100.0 * (ooo_cpu[i]->num_branch - ooo_cpu[i]->branch_mispredictions)) / ooo_cpu[i]->num_branch;
    cout << "% MPKI: " << (1000.0 * ooo_cpu[i]->branch_mispredictions) / (ooo_cpu[i]->num_retired - warmup_instructions);
    cout << " Average ROB Occupancy at Mispredict: " << (1.0 * ooo_cpu[i]->total_rob_occupancy_at_branch_mispredict) / ooo_cpu[i]->branch_mispredictions
         << endl;

    /*
    cout << "Branch types" << endl;
    cout << "NOT_BRANCH: " << ooo_cpu[i]->total_branch_types[0] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[0])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_DIRECT_JUMP: "
    << ooo_cpu[i]->total_branch_types[1] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[1])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_INDIRECT: " <<
    ooo_cpu[i]->total_branch_types[2] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[2])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_CONDITIONAL: "
    << ooo_cpu[i]->total_branch_types[3] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[3])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_DIRECT_CALL: "
    << ooo_cpu[i]->total_branch_types[4] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[4])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_INDIRECT_CALL:
    " << ooo_cpu[i]->total_branch_types[5] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[5])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_RETURN: " <<
    ooo_cpu[i]->total_branch_types[6] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[6])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl; cout << "BRANCH_OTHER: " <<
    ooo_cpu[i]->total_branch_types[7] << " " <<
    (100.0*ooo_cpu[i]->total_branch_types[7])/(ooo_cpu[i]->num_retired -
    ooo_cpu[i]->begin_sim_instr) << "%" << endl << endl;
    */

    cout << "Branch type MPKI" << endl;
    cout << "BRANCH_DIRECT_JUMP: " << (1000.0 * ooo_cpu[i]->branch_type_misses[1] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl;
    cout << "BRANCH_INDIRECT: " << (1000.0 * ooo_cpu[i]->branch_type_misses[2] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl;
    cout << "BRANCH_CONDITIONAL: " << (1000.0 * ooo_cpu[i]->branch_type_misses[3] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl;
    cout << "BRANCH_DIRECT_CALL: " << (1000.0 * ooo_cpu[i]->branch_type_misses[4] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl;
    cout << "BRANCH_INDIRECT_CALL: " << (1000.0 * ooo_cpu[i]->branch_type_misses[5] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl;
    cout << "BRANCH_RETURN: " << (1000.0 * ooo_cpu[i]->branch_type_misses[6] / (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) << endl << endl;
  }
}

 void print_dram_stats()
 {
   uint64_t total_congested_cycle = 0;
   uint64_t total_congested_count = 0;

   std::cout << std::endl;
   std::cout << "DRAM Statistics" << std::endl;
   for (uint32_t i = 0; i < DRAM_CHANNELS; i++) {
     std::cout << " CHANNEL " << i << std::endl;
    
     auto& channel = DRAM.channels[i];
     std::cout << " RQ ROW_BUFFER_HIT: " << std::setw(10) << channel.RQ_ROW_BUFFER_HIT << " ";
     std::cout << " ROW_BUFFER_MISS: " << std::setw(10) << channel.RQ_ROW_BUFFER_MISS;
     std::cout << std::endl;

     std::cout << " DBUS AVG_CONGESTED_CYCLE: ";
     if (channel.dbus_count_congested)
       std::cout << std::setw(10) << ((double)channel.dbus_cycle_congested / channel.dbus_count_congested);
     else
       std::cout << "-";
     std::cout << std::endl;

     std::cout << " WQ ROW_BUFFER_HIT: " << std::setw(10) << channel.WQ_ROW_BUFFER_HIT << " ";
     std::cout << " ROW_BUFFER_MISS: " << std::setw(10) << channel.WQ_ROW_BUFFER_MISS << " ";
     std::cout << " FULL: " << std::setw(10) << channel.WQ_FULL;
     std::cout << std::endl;

     std::cout << std::endl;

     total_congested_cycle += channel.dbus_cycle_congested;
     total_congested_count += channel.dbus_count_congested;

   if (DRAM_CHANNELS > 1) {
     std::cout << " DBUS AVG_CONGESTED_CYCLE: ";
     if (total_congested_count)
       std::cout << std::setw(10) << ((double)total_congested_cycle / total_congested_count);
     else
       std::cout << "-";
     std::cout << std::endl;
   }
 }
}

void reset_cache_stats(uint32_t cpu, CACHE* cache)
{
  for (uint32_t i = 0; i < NUM_TYPES; i++) {
    cache->sim_access[cpu][i] = 0;
    cache->sim_hit[cpu][i] = 0;
    cache->sim_miss[cpu][i] = 0;
  }

  cache->pf_requested = 0;
  cache->pf_issued = 0;
  cache->pf_useful = 0;
  cache->pf_useless = 0;
  cache->pf_fill = 0;

  cache->total_miss_latency = 0;

  cache->RQ_ACCESS = 0;
  cache->RQ_MERGED = 0;
  cache->RQ_TO_CACHE = 0;

  cache->WQ_ACCESS = 0;
  cache->WQ_MERGED = 0;
  cache->WQ_TO_CACHE = 0;
  cache->WQ_FORWARD = 0;
  cache->WQ_FULL = 0;
}

void finish_warmup()
{
  uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time), elapsed_minute = elapsed_second / 60, elapsed_hour = elapsed_minute / 60;
  elapsed_minute -= elapsed_hour * 60;
  elapsed_second -= (elapsed_hour * 3600 + elapsed_minute * 60);

  // reset core latency
  // note: since re-ordering he function calls in the main simulation loop, it's
  // no longer necessary to add
  //       extra latency for scheduling and execution, unless you want these
  //       steps to take longer than 1 cycle.
  // PAGE_TABLE_LATENCY = 100;
  // SWAP_LATENCY = 100000;

  cout << endl;
  for (uint32_t i = 0; i < NUM_CPUS; i++) {
    for (int j=0; j<traces.size(); j++) {
      cout << "Warmup complete CPU " << i << " instructions from smt thread " << j << " : " << ooo_cpu[i]->smt_num_retired[j] << " cycles: " << ooo_cpu[i]->current_cycle << endl;
    }
    cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;

    ooo_cpu[i]->begin_sim_cycle = ooo_cpu[i]->current_cycle;
    ooo_cpu[i]->begin_sim_instr = ooo_cpu[i]->num_retired;

    // reset branch stats
    ooo_cpu[i]->num_branch = 0;
    ooo_cpu[i]->branch_mispredictions = 0;
    ooo_cpu[i]->total_rob_occupancy_at_branch_mispredict = 0;

    for (uint32_t j = 0; j < 8; j++) {
      ooo_cpu[i]->total_branch_types[j] = 0;
      ooo_cpu[i]->branch_type_misses[j] = 0;
    }

    bzero(&ooo_cpu[i]->stats, sizeof(ooo_cpu[i]->stats));
    for(uint32_t index = 0; index < 1; ++index)
    {
      for(uint32_t smt_id=0; smt_id<traces.size(); smt_id++) {
        ooo_cpu[i]->bubble_max[smt_id] = 0;
        ooo_cpu[i]->bubble_min[smt_id] = UINT64_MAX;
        ooo_cpu[i]->bubble_tot[smt_id] = 0;
        ooo_cpu[i]->bubble_cnt[smt_id] = 0;
      }

    }

    ooo_cpu[i]->load_per_ip_stats.clear();
    ooo_cpu[i]->frontal_load_per_ip_stats.clear();
    for(uint32_t index = 0; index < 1; ++index)
        ooo_cpu[i]->load_per_rob_part_stats.reset();

    ooo_cpu[i]->glob_mlp_amount.reset();
    ooo_cpu[i]->min_mlp_dist.reset();
    ooo_cpu[i]->max_mlp_dist.reset();
    for (int j=0; j<traces.size(); j++) {
      ooo_cpu[i]->mlp_amount[j].reset();
    }

    for (auto it = caches.rbegin(); it != caches.rend(); ++it)
      reset_cache_stats(i, *it);
  }
  cout << endl;

  // reset DRAM stats
  // for (uint32_t i = 0; i < DRAM_CHANNELS; i++) {
  //   DRAM.channels[i].WQ_ROW_BUFFER_HIT = 0;
  //   DRAM.channels[i].WQ_ROW_BUFFER_MISS = 0;
  //   DRAM.channels[i].RQ_ROW_BUFFER_HIT = 0;
  //   DRAM.channels[i].RQ_ROW_BUFFER_MISS = 0;
  // }
}

void signal_handler(int signal)
{
  cout << "Caught signal: " << signal << endl;
  exit(1);
}

int main(int argc, char** argv)
{
  // interrupt signal hanlder
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  cout << endl << "*** ChampSim Multicore Out-of-Order Simulator ***" << endl << endl;

  // initialize knobs
  uint8_t show_heartbeat = 1;

  // check to see if knobs changed using getopt_long()
  int traces_encountered = 0;
  static struct option long_options[] = {{"warmup_instructions", required_argument, 0, 'w'},
                                         {"simulation_instructions", required_argument, 0, 'i'},
                                         {"hide_heartbeat", no_argument, 0, 'h'},
                                         {"cloudsuite", no_argument, 0, 'c'},
                                         {"traces", no_argument, &traces_encountered, 1},
                                         {"switch_policy", required_argument, 0, 'p'},
                                         {0, 0, 0, 0}};

  int c;
  while ((c = getopt_long_only(argc, argv, "w:i:hc", long_options, NULL)) != -1 && !traces_encountered) {
    switch (c) {
    case 'w':
      warmup_instructions = atol(optarg);
      break;
    case 'i':
      simulation_instructions = atol(optarg);
      break;
    case 'h':
      show_heartbeat = 0;
      break;
    case 'c':
      knob_cloudsuite = 1;
      MAX_INSTR_DESTINATIONS = NUM_INSTR_DESTINATIONS_SPARC;
      break;
    case 'p':
      switch_policy = atol(optarg);
      break;
    case 0:
      break;
    default:
      abort();
    }
  }

  cout << "Warmup Instructions: " << warmup_instructions << endl;
  cout << "Simulation Instructions: " << simulation_instructions << endl;
  cout << "Number of CPUs: " << NUM_CPUS << endl;

  // long long int dram_size = DRAM_CHANNELS * DRAM_RANKS * DRAM_BANKS * DRAM_ROWS * DRAM_COLUMNS * BLOCK_SIZE / 1024 / 1024; // in MiB
  // std::cout << "Off-chip DRAM Size: ";
  // if (dram_size > 1024)
  //   std::cout << dram_size / 1024 << " GiB";
  // else
  //   std::cout << dram_size << " MiB";
  // std::cout << " Channels: " << DRAM_CHANNELS << " Width: " << 8 * DRAM_CHANNEL_WIDTH << "-bit Data Rate: " << DRAM_IO_FREQ << " MT/s" << std::endl;
  std::cout << " CXL_MODE: " << CXL_MODE << std::endl;
  std::cout << std::endl;
  std::cout << "VirtualMemory physical capacity: " << std::size(vmem.ppage_free_list) * vmem.page_size;
  std::cout << " num_ppages: " << std::size(vmem.ppage_free_list) << std::endl;
  std::cout << "VirtualMemory page size: " << PAGE_SIZE << " log2_page_size: " << LOG2_PAGE_SIZE << std::endl;

  std::cout << std::endl;
  for (int i = optind; i < argc; i++) {
    //std::cout << "CPU " << traces.size() << " runs " << argv[i] << std::endl;

    traces.push_back(get_tracereader(argv[i], traces.size(), knob_cloudsuite));
    /*
    if (traces.size() > NUM_CPUS) {
      printf("\n*** Too many traces for the configured number of cores ***\n\n");
      assert(0);
    }
    */
  }

  /*
  if (traces.size() != NUM_CPUS) {
    printf("\n*** Not enough traces for the configured number of cores ***\n\n");
    assert(0);
  }
  */
  // end trace file setup

  // SHARED CACHE
  for (O3_CPU* cpu : ooo_cpu) {
    cpu->initialize_core();
    cpu->num_traces = traces.size();
  }

  for (auto it = caches.rbegin(); it != caches.rend(); ++it) {
    (*it)->impl_prefetcher_initialize();
    (*it)->impl_replacement_initialize();
  }

  uint32_t cur_trace_id = 0;

  // simulation entry point
  while (std::any_of(std::begin(simulation_complete), std::end(simulation_complete), std::logical_not<uint8_t>())) {

    uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time), elapsed_minute = elapsed_second / 60, elapsed_hour = elapsed_minute / 60;
    elapsed_minute -= elapsed_hour * 60;
    elapsed_second -= (elapsed_hour * 3600 + elapsed_minute * 60);

    for (auto op : operables) {
      try {
        op->_operate();
      } catch (champsim::deadlock& dl) {
        // ooo_cpu[dl.which]->print_deadlock();
        // std::cout << std::endl;
        // for (auto c : caches)
        for (auto c : operables) {
          c->print_deadlock();
          std::cout << std::endl;
        }

        abort();
      }
    }
    std::sort(std::begin(operables), std::end(operables), champsim::by_next_operate());

    for (std::size_t i = 0; i < ooo_cpu.size(); ++i) {
      // read from trace

      while (ooo_cpu[i]->instrs_to_read_this_cycle > 0) {

        uint64_t start = ooo_cpu[i]->instrs_to_read_this_cycle;
        uint32_t minim = UINT32_MAX, min_it = UINT32_MAX;
        
        if (!warmup_complete[i]) {
          uint32_t temp = cur_trace_id;
          if (ooo_cpu[i]->fetch_stall[temp] == 0 && ooo_cpu[i]->smt_instrs_to_read_this_cycle[temp]) {
            min_it = cur_trace_id;
            cur_trace_id = (cur_trace_id+1)%traces.size();
          }
          else {
            uint count = traces.size();
            while (count > 0) {
              if (ooo_cpu[i]->fetch_stall[temp] == 0 && ooo_cpu[i]->smt_instrs_to_read_this_cycle[temp]) {
                min_it = cur_trace_id;
                cur_trace_id = (cur_trace_id+1)%traces.size();
                break;
              }
              else {
                temp = (temp+1)%traces.size();
                count--;
              }
            }
          }
        }

        else {
          switch (switch_policy) {
            case RR:
            {
              uint32_t temp = cur_trace_id;
              if (ooo_cpu[i]->fetch_stall[temp] == 0) {
                min_it = cur_trace_id;
                cur_trace_id = (cur_trace_id+1)%traces.size();
              }
              else {
                uint count = traces.size();
                while (count > 0) {
                  if (ooo_cpu[i]->fetch_stall[temp] == 0) {
                    min_it = cur_trace_id;
                    cur_trace_id = (cur_trace_id+1)%traces.size();
                    break;
                  }
                  else {
                    temp = (temp+1)%traces.size();
                    count--;
                  }
                }
              }
            }
              break;
            case ICOUNT: 
            {
              for (uint32_t smt_id = 0; smt_id<traces.size(); smt_id++) {
                // if warmup isn't complete and this thread also hasn't completed warmup
                // of ir warmup is complete but this thread still has roi instructions
                if (!smt_simulation_complete[smt_id]) { 
                  if(ooo_cpu[i]->fetch_stall[smt_id] == 0 && minim > ooo_cpu[i]->ROB[smt_id].occupancy() && ooo_cpu[i]->smt_instrs_to_read_this_cycle[smt_id]) {
                    minim = ooo_cpu[i]->ROB[smt_id].occupancy();
                    min_it = smt_id;
                  }
                }
              }
            }
              break;
            case ON_DETECT_MISS:
              break;
            default:
              break;
          }

        }

        if(min_it != UINT32_MAX) {
          ooo_cpu[i]->init_instruction(traces[min_it]->get(), min_it);
        }
        if (start == ooo_cpu[i]->instrs_to_read_this_cycle) {
          break;
        }

      }

      // heartbeat information     
      if (show_heartbeat && (ooo_cpu[i]->num_retired >= ooo_cpu[i]->next_print_instruction)) {
        float cumulative_ipc;
        if (warmup_complete[i])
          cumulative_ipc = (1.0 * (ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr)) / (ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle);
        else
          cumulative_ipc = (1.0 * ooo_cpu[i]->num_retired) / ooo_cpu[i]->current_cycle;
        float heartbeat_ipc = (1.0 * ooo_cpu[i]->num_retired - ooo_cpu[i]->last_sim_instr) / (ooo_cpu[i]->current_cycle - ooo_cpu[i]->last_sim_cycle);

        cout << "Heartbeat CPU " << i << " instructions: " << ooo_cpu[i]->num_retired << " cycles: " << ooo_cpu[i]->current_cycle;
        cout << " heartbeat IPC: " << heartbeat_ipc << " cumulative IPC: " << cumulative_ipc;
        cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;
        for(int j=0; j<traces.size(); j++) {
          cout << "Heartbeat SMT " << j << " instructions: " << ooo_cpu[i]->smt_num_retired[j] << " cycles: " << ooo_cpu[i]->current_cycle << endl;
        }
        ooo_cpu[i]->next_print_instruction += STAT_PRINTING_PERIOD;

        ooo_cpu[i]->last_sim_instr = ooo_cpu[i]->num_retired;
        ooo_cpu[i]->last_sim_cycle = ooo_cpu[i]->current_cycle;
      }

      // check for warmup
      // warmup complete
      if (warmup_complete[i] == 0) {
        
        for (int j=0; j<traces.size(); j++) {
          if (ooo_cpu[i]->smt_num_retired[j] > warmup_instructions) {
            smt_warmup_complete[j] = 1;
            smt_all_warmup_complete[i]++;
          }
        }
      
        if (smt_all_warmup_complete[i] == traces.size()) {
        //if(ooo_cpu[i]->num_retired > warmup_instructions) {
          warmup_complete[i] = 1;
          all_warmup_complete++;
          for (int j=0; j<traces.size(); j++) {
            ooo_cpu[i]->smt_begin_sim_instr[j] =  ooo_cpu[i]->smt_num_retired[j];
          }
        }     
      }

      if (all_warmup_complete == NUM_CPUS) { // this part is called only once
                                             // when all cores are warmed up
        all_warmup_complete++;
        finish_warmup();
      }

      // simulation complete
      if ((all_warmup_complete > NUM_CPUS) && (simulation_complete[i] == 0)){
      
        for (int j=0; j<traces.size(); j++) {
          if (ooo_cpu[i]->smt_num_retired[j] >= (ooo_cpu[i]->smt_begin_sim_instr[j] + simulation_instructions)) {
            smt_simulation_complete[j] = 1;
            smt_all_simulation_complete[i]++;
            
            ooo_cpu[i]->smt_finish_sim_instr[j] = ooo_cpu[i]->smt_num_retired[j] - ooo_cpu[i]->smt_begin_sim_instr[j];
            ooo_cpu[i]->smt_finish_sim_cycle[j] = ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle;
            
            //cout << "Finished smt thread  " << j << " instructions: " << ooo_cpu[i]->smt_finish_sim_instr[j] << " cycles: " << ooo_cpu[i]->smt_finish_sim_cycle[j];
            //cout << " smt IPC: " << ((float)ooo_cpu[i]->smt_finish_sim_instr[j] / ooo_cpu[i]->smt_finish_sim_cycle[j]);
            //cout << endl;
          }
        } 
        
        if (smt_all_simulation_complete[i] == traces.size()) {
        
        //if(ooo_cpu[i]->num_retired >= (ooo_cpu[i]->begin_sim_instr + simulation_instructions)) {
          simulation_complete[i] = 1;
          ooo_cpu[i]->finish_sim_instr = ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr;
          ooo_cpu[i]->finish_sim_cycle = ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle;

          cout << "Finished CPU " << i << " instructions: " << ooo_cpu[i]->finish_sim_instr << " cycles: " << ooo_cpu[i]->finish_sim_cycle;
          cout << " cumulative IPC: " << ((float)ooo_cpu[i]->finish_sim_instr / ooo_cpu[i]->finish_sim_cycle);
          cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;

          for (int j=0; j<traces.size(); j++) {         
              ooo_cpu[i]->smt_finish_sim_instr[j] = ooo_cpu[i]->smt_num_retired[j] - ooo_cpu[i]->smt_begin_sim_instr[j];
              ooo_cpu[i]->smt_finish_sim_cycle[j] = ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle;
              
              cout << "Finished smt thread  " << j << " instructions: " << ooo_cpu[i]->smt_finish_sim_instr[j] << " cycles: " << ooo_cpu[i]->smt_finish_sim_cycle[j];
              cout << " smt IPC: " << ((float)ooo_cpu[i]->smt_finish_sim_instr[j] / ooo_cpu[i]->smt_finish_sim_cycle[j]);
              cout << endl;
            }

          for (auto it = caches.rbegin(); it != caches.rend(); ++it)
            record_roi_stats(i, *it);
        }
      }
    }
  }

  uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time), elapsed_minute = elapsed_second / 60, elapsed_hour = elapsed_minute / 60;
  elapsed_minute -= elapsed_hour * 60;
  elapsed_second -= (elapsed_hour * 3600 + elapsed_minute * 60);

  cout << endl << "ChampSim completed all CPUs" << endl;
  if (NUM_CPUS > 1) {
    cout << endl << "Total Simulation Statistics (not including warmup)" << endl;
    for (uint32_t i = 0; i < NUM_CPUS; i++) {
      cout << endl
           << "CPU " << i
           << " cumulative IPC: " << (float)(ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr) / (ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle);
      cout << " instructions: " << ooo_cpu[i]->num_retired - ooo_cpu[i]->begin_sim_instr
           << " cycles: " << ooo_cpu[i]->current_cycle - ooo_cpu[i]->begin_sim_cycle << endl;
      for (auto it = caches.rbegin(); it != caches.rend(); ++it)
        print_sim_stats(i, *it);
    }
  }

  cout << endl << "Region of Interest Statistics" << endl;
  for (uint32_t i = 0; i < NUM_CPUS; i++) {
    cout << endl << "CPU " << i << " cumulative IPC: " << ((float)ooo_cpu[i]->finish_sim_instr / ooo_cpu[i]->finish_sim_cycle);
    cout << " instructions: " << ooo_cpu[i]->finish_sim_instr << " cycles: " << ooo_cpu[i]->finish_sim_cycle << endl;
    for (auto it = caches.rbegin(); it != caches.rend(); ++it)
      print_roi_stats(i, *it);
    print_cumulative_stats(i);
  }

  for (auto it = caches.rbegin(); it != caches.rend(); ++it)
    (*it)->impl_prefetcher_final_stats();

  for (auto it = caches.rbegin(); it != caches.rend(); ++it)
    (*it)->impl_replacement_final_stats();

std::cout << "printing mem stats" << std::endl;
#ifndef CRC2_COMPILE
  print_dram_stats();
  //DRAM.PrintStats();
  print_branch_stats();
#endif

  return 0;
}
