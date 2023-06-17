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

#ifndef OOO_CPU_H
#define OOO_CPU_H

#include <array>
#include <functional>
#include <queue>

#include "block.h"
#include "champsim.h"
#include "delay_queue.hpp"
#include "instruction.h"
#include "memory_class.h"
#include "operable.h"
#include <list>
#include <cstring>

using namespace std;

class GlobAlloc {
    public:
        virtual ~GlobAlloc() {}

        inline void* operator new (size_t sz) {
            return malloc(sz);
        }

        //Placement new
        inline void* operator new (size_t sz, void* ptr) {
            return ptr;
        }

        inline void operator delete(void *p, size_t sz) {
            free(p);
        }

        //Placement delete... make ICC happy. This would only fire on an exception
        void operator delete (void* p, void* ptr) {}
};

class Stat : public GlobAlloc {
    protected:
        const char* _name;
        const char* _desc;

    public:
        Stat() : _name(nullptr), _desc(nullptr) {}

        virtual ~Stat() {}

        const char* name() const {
            assert(_name);
            return _name;
        }

        const char* desc() const {
            assert(_desc);
            return _desc;
        }

    protected:
        virtual void initStat(const char* name, const char* desc) {
            assert(name);
            assert(desc);
            assert(!_name);
            assert(!_desc);
            _name = name;
            _desc = desc;
        }
};

class VectorStat : public Stat {
    protected:
        const char** _counterNames;

    public:
        VectorStat() : _counterNames(nullptr) {}

        virtual uint64_t count(uint32_t idx) const = 0;
        virtual uint32_t size() const = 0;

        inline bool hasCounterNames() {
            return (_counterNames != nullptr);
        }

        inline const char* counterName(uint32_t idx) const {
            return (_counterNames == nullptr)? nullptr : _counterNames[idx];
        }

        virtual void init(const char* name, const char* desc) {
            initStat(name, desc);
        }
};

template <typename T> T* gm_dup(T* src, size_t objs) {
    T* dst = (T*)malloc(sizeof(T) * objs);
    memcpy(dst, src, sizeof(T)*objs);
    return dst;
}

class VectorCounter : public VectorStat {
    private:
        std::vector<uint64_t> _counters;

    public:
        VectorCounter() : VectorStat() {}

        /* Without counter names */
        virtual void init(const char* name, const char* desc, uint32_t size) {
            initStat(name, desc);
            assert(size > 0);
            _counters.resize(size);
            for (uint32_t i = 0; i < size; i++) _counters[i] = 0;
            _counterNames = nullptr;
        }

        /* With counter names */
        virtual void init(const char* name, const char* desc, uint32_t size, const char** counterNames) {
            init(name, desc, size);
            assert(counterNames);
            _counterNames = gm_dup(counterNames, size);
        }

        inline void inc(uint32_t idx, uint64_t value) {
            _counters[idx] += value;
        }

        inline void inc(uint32_t idx) {
             _counters[idx]++;
        }

        inline void atomicInc(uint32_t idx, uint64_t delta) {
            __sync_fetch_and_add(&_counters[idx], delta);
        }

        inline void atomicInc(uint32_t idx) {
            __sync_fetch_and_add(&_counters[idx], 1);
        }

        inline virtual uint64_t count(uint32_t idx) const {
            return _counters[idx];
        }

        inline uint32_t size() const {
            return _counters.size();
        }

        inline void reset() {
            for (uint32_t i = 0; i < _counters.size(); i++) _counters[i] = 0;
        }
};

class load_per_ip_info_t
{
public:
    uint64_t total_loads;
    uint64_t loads_went_offchip;
    uint64_t loads_went_offchip_pos_hist; // histogram of offchip loads based on dispatch position
    load_per_ip_info_t()
    {
        total_loads = 0;
        loads_went_offchip = 0;
    }
};

class load_per_rob_part_info_t
{
public:
    uint64_t total_loads;
    uint64_t loads_went_offchip;
    void reset()
    {
        total_loads = 0;
        loads_went_offchip = 0;        
    }
    load_per_rob_part_info_t() {reset();}
};

class CACHE;

class CacheBus : public MemoryRequestProducer
{
public:
  champsim::circular_buffer<PACKET> PROCESSED;
  CacheBus(std::size_t q_size, MemoryRequestConsumer* ll) : MemoryRequestProducer(ll), PROCESSED(q_size) {}
  void return_data(PACKET* packet);
};

// cpu
class O3_CPU : public champsim::operable
{
public:
  uint32_t cpu = 0;

  // instruction
  uint64_t instr_unique_id = 1, completed_executions = 0, begin_sim_cycle = 0, begin_sim_instr = 0, smt_begin_sim_instr[8] = {}, last_sim_cycle = 0, last_sim_instr = 0,
           finish_sim_cycle = 0, smt_finish_sim_cycle[8] = {}, finish_sim_instr = 0, smt_finish_sim_instr[8] = {}, instrs_to_read_this_cycle = 0, smt_instrs_to_read_this_cycle[8] = {}, instrs_to_fetch_this_cycle = 0,
           next_print_instruction = STAT_PRINTING_PERIOD, num_retired = 0, smt_num_retired[8] = {};
  uint32_t inflight_reg_executions = 0, inflight_mem_executions = 0;

  struct dib_entry_t {
    bool valid = false;
    unsigned lru = 999999;
    uint64_t address = 0;
    uint32_t trace_id = 100000;
  };

  // instruction buffer
  using dib_t = std::vector<dib_entry_t>;
  const std::size_t dib_set, dib_way, dib_window;
  dib_t DIB{dib_set * dib_way};

  // reorder buffer, load/store queue, register file
  
  std::vector<champsim::circular_buffer<ooo_model_instr>> IFETCH_BUFFER;  
  std::vector<champsim::delay_queue<ooo_model_instr>> DISPATCH_BUFFER;
  std::vector<champsim::delay_queue<ooo_model_instr>> DECODE_BUFFER;

  //champsim::circular_buffer<ooo_model_instr> IFETCH_BUFFER;
  //champsim::delay_queue<ooo_model_instr> DISPATCH_BUFFER;
  //champsim::delay_queue<ooo_model_instr> DECODE_BUFFER;
  std::vector<champsim::circular_buffer<ooo_model_instr>> ROB;
  std::vector<std::vector<LSQ_ENTRY>> LQ;
  std::vector<std::vector<LSQ_ENTRY>> SQ;

  // Constants
  const unsigned FETCH_WIDTH, DECODE_WIDTH, DISPATCH_WIDTH, SCHEDULER_SIZE, EXEC_WIDTH, LQ_WIDTH, SQ_WIDTH, RETIRE_WIDTH;
  const unsigned BRANCH_MISPREDICT_PENALTY, SCHEDULING_LATENCY, EXEC_LATENCY;

  // store array, this structure is required to properly handle store
  // instructions
  std::deque<uint64_t> STA[8];

  // Ready-To-Execute
  std::queue<champsim::circular_buffer<ooo_model_instr>::iterator> ready_to_execute;

  // Ready-To-Load
  std::queue<std::vector<LSQ_ENTRY>::iterator> RTL0, RTL1;

  // Ready-To-Store
  std::queue<std::vector<LSQ_ENTRY>::iterator> RTS0, RTS1;

  // branch
  uint8_t fetch_stall[8] = {};
  uint64_t fetch_resume_cycle[8] = {};
  uint64_t num_branch = 0, branch_mispredictions = 0;
  uint64_t total_rob_occupancy_at_branch_mispredict;

  uint64_t total_branch_types[8] = {};
  uint64_t branch_type_misses[8] = {};

  uint32_t num_traces;
  uint64_t glob_rob_occupancy = 0, glob_sq_occupancy = 0, glob_lq_occupancy = 0;
  uint64_t glob_ifetch_occupancy = 0, glob_dispatch_occupancy = 0, glob_decode_occupancy = 0;

  CacheBus ITLB_bus, DTLB_bus, L1I_bus, L1D_bus;

  struct
  {
      struct
      {
          uint64_t called;
          uint64_t rob_non_head;
          uint64_t rob_head;
          uint64_t went_offchip;
          uint64_t went_offchip_rob_head;
          uint64_t went_offchip_rob_non_head;
      } bubble[8];

      struct
      {
          uint64_t pred_called;
          uint64_t true_pos;
          uint64_t false_pos;
          uint64_t false_neg;
      } offchip_pred;

      struct
      {
          uint64_t total;
          uint64_t issued[2];
          uint64_t dram_rq_full;
          uint64_t dram_mshr_full;
      } ddrp;

  } stats;

  // bubble stats per ROB partiton
  uint64_t bubble_max[8], bubble_min[8], bubble_tot[8], bubble_cnt[8];

  unordered_map<uint64_t, load_per_ip_info_t> load_per_ip_stats, frontal_load_per_ip_stats;
  load_per_rob_part_info_t load_per_rob_part_stats;


  VectorCounter mlp_amount[8], max_mlp_dist, min_mlp_dist, glob_mlp_amount;

  void operate();

  // functions
  void init_instruction(ooo_model_instr instr, uint32_t smt_id);
  void check_dib();
  void translate_fetch();
  void fetch_instruction();
  void promote_to_decode();
  void decode_instruction();
  void dispatch_instruction();
  void schedule_instruction();
  void execute_instruction();
  void schedule_memory_instruction();
  void execute_memory_instruction();
  void do_check_dib(ooo_model_instr& instr);
  void do_translate_fetch(champsim::circular_buffer<ooo_model_instr>::iterator begin, champsim::circular_buffer<ooo_model_instr>::iterator end, uint32_t smt_id);
  void do_fetch_instruction(champsim::circular_buffer<ooo_model_instr>::iterator begin, champsim::circular_buffer<ooo_model_instr>::iterator end);
  void do_dib_update(const ooo_model_instr& instr);
  void do_scheduling(champsim::circular_buffer<ooo_model_instr>::iterator rob_it, uint32_t smt_id);
  void do_execution(champsim::circular_buffer<ooo_model_instr>::iterator rob_it);
  void do_memory_scheduling(champsim::circular_buffer<ooo_model_instr>::iterator rob_it, uint32_t smt_id);
  void operate_lsq();
  void do_complete_execution(champsim::circular_buffer<ooo_model_instr>::iterator rob_it);
  void do_sq_forward_to_lq(LSQ_ENTRY& sq_entry, LSQ_ENTRY& lq_entry);

  void initialize_core();
  void add_load_queue(champsim::circular_buffer<ooo_model_instr>::iterator rob_index, uint32_t data_index, uint32_t smt_id);
  void add_store_queue(champsim::circular_buffer<ooo_model_instr>::iterator rob_index, uint32_t data_index, uint32_t smt_id);
  void execute_store(std::vector<LSQ_ENTRY>::iterator sq_it);
  int execute_load(std::vector<LSQ_ENTRY>::iterator lq_it);
  int do_translate_store(std::vector<LSQ_ENTRY>::iterator sq_it);
  int do_translate_load(std::vector<LSQ_ENTRY>::iterator lq_it);
  void check_dependency(int prior, int current);
  void operate_cache();
  void complete_inflight_instruction();
  void handle_memory_return();
  void retire_rob();

  void print_deadlock() override;

  int prefetch_code_line(uint64_t pf_v_addr);

  void measure_pipeline_bubble_stats(std::vector<LSQ_ENTRY>::iterator lq_index, champsim::circular_buffer<ooo_model_instr>::iterator rob_index);
  void monitor_loads(std::vector<LSQ_ENTRY>::iterator lq_index);

#include "ooo_cpu_modules.inc"

  const bpred_t bpred_type;
  const btb_t btb_type;
  const ipref_t ipref_type;

  O3_CPU(uint32_t cpu, double freq_scale, std::size_t dib_set, std::size_t dib_way, std::size_t dib_window, std::size_t ifetch_buffer_size,
         std::size_t decode_buffer_size, std::size_t dispatch_buffer_size, std::size_t rob_size, std::size_t lq_size, std::size_t sq_size, unsigned fetch_width,
         unsigned decode_width, unsigned dispatch_width, unsigned schedule_width, unsigned execute_width, unsigned lq_width, unsigned sq_width,
         unsigned retire_width, unsigned mispredict_penalty, unsigned decode_latency, unsigned dispatch_latency, unsigned schedule_latency,
         unsigned execute_latency, MemoryRequestConsumer* itlb, MemoryRequestConsumer* dtlb, MemoryRequestConsumer* l1i, MemoryRequestConsumer* l1d,
         bpred_t bpred_type, btb_t btb_type, ipref_t ipref_type)
      : champsim::operable(freq_scale), cpu(cpu), dib_set(dib_set), dib_way(dib_way), dib_window(dib_window), 
        IFETCH_BUFFER(8, champsim::circular_buffer<ooo_model_instr>(ifetch_buffer_size)),
        DISPATCH_BUFFER(8, champsim::delay_queue<ooo_model_instr>(dispatch_buffer_size, dispatch_latency)), 
        DECODE_BUFFER(8, champsim::delay_queue<ooo_model_instr>(decode_buffer_size, decode_latency)), 
        //IFETCH_BUFFER(ifetch_buffer_size),
        //DISPATCH_BUFFER(dispatch_buffer_size, dispatch_latency), 
        //DECODE_BUFFER(decode_buffer_size, decode_latency), 
        ROB(8, champsim::circular_buffer<ooo_model_instr>(rob_size)), 
        LQ(8, std::vector<LSQ_ENTRY>(lq_size)), SQ(8, std::vector<LSQ_ENTRY>(sq_size)),
        FETCH_WIDTH(fetch_width), DECODE_WIDTH(decode_width), DISPATCH_WIDTH(dispatch_width), SCHEDULER_SIZE(schedule_width), EXEC_WIDTH(execute_width),
        LQ_WIDTH(lq_width), SQ_WIDTH(sq_width), RETIRE_WIDTH(retire_width), BRANCH_MISPREDICT_PENALTY(mispredict_penalty), SCHEDULING_LATENCY(schedule_latency),
        EXEC_LATENCY(execute_latency), ITLB_bus(rob_size, itlb), DTLB_bus(rob_size, dtlb), L1I_bus(rob_size, l1i), L1D_bus(rob_size, l1d),
        bpred_type(bpred_type), btb_type(btb_type), ipref_type(ipref_type)
  {
    for(int i=0; i<8; i++) {
      string result = "smt_thread_" + to_string(i) + "_local_mlp_amount";
      mlp_amount[i].init(result.c_str(), "Number of long latency load instances with X amount of mlp within the same thread", 64);
    }
    glob_mlp_amount.init("mlp_amount_across_threads", "Number of long latency load instances with X amount of mlp across threads", 64);
    
    min_mlp_dist.init("min_mlp_dist", "Number of long latency load instances with the next lld at distance X", 11);
    max_mlp_dist.init("max_mlp_amount", "Number of long latency load instances with the last lld at distance X", 11);
  }
};

#endif
