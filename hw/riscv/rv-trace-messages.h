/*
 * Helpers for RISC-V Trace Messages
 *
 * Copyright (C) 2025 Ventana Micro Systems Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef RISCV_RV_TRACE_MESSAGES_H
#define RISCV_RV_TRACE_MESSAGES_H

typedef enum {
    U = 0,
    S_HS = 1,
    RESERVED = 2,
    M = 3,
    D = 4,
    VU = 5,
    VS = 6,
} TracePrivLevel;

size_t rv_etrace_gen_encoded_sync_msg(uint8_t *buf, uint64_t pc,
                                      TracePrivLevel priv_level,
                                      bool pc_is_branch);
size_t rv_etrace_gen_encoded_trap_msg(uint8_t *buf, uint64_t trap_addr,
                                      TracePrivLevel priv_level,
                                      uint8_t ecause,
                                      bool is_interrupt,
                                      uint64_t tval);
size_t rv_etrace_gen_encoded_format2_msg(uint8_t *buf, uint64_t addr,
                                         bool notify, bool updiscon);
size_t rv_etrace_gen_encoded_format1_noaddr(uint8_t *buf,
                                            uint8_t branches,
                                            uint32_t branch_map);
size_t rv_etrace_gen_encoded_format1(uint8_t *buf,
                                     uint8_t branches, uint32_t branch_map,
                                     uint64_t addr,
                                     bool notify, bool updiscon);


/* Taken from rv_etrace/include/rv_etrace_params.h */
struct rv_etrace_packet_params {
    /* Number of bytes for source ID (maximum 2) */
    unsigned int srcid_bytes_p;
    /* Number of timestamp bytes (maximum 8) */
    unsigned int tstamp_bytes_p;
    /* Number of type bits */
    unsigned int type_width_p;
};

/* Payload level intruction trace paratmeters */
struct rv_etrace_itrace_params {
    /*
     * The architecture specification version with which the encoder
     * is compliant (0 for initial version).
     */
    unsigned int arch_p;
    /* Number of times iretire, itype etc. are replicated */
    unsigned int blocks_p;
    /*
     * Number of entries in the branch predictor is 2bpred_size_p.
     * Minimum number of entries is 2, so a value of 0 indicates
     * that there is no branch predictor implemented.
     */
    unsigned int bpred_size_p;
    /*
     * Number of entries in the jump target cache is 2cache_size_p.
     * Minimum number of entries is 2, so a value of 0 indicates
     * that there is no jump target cache implemented.
     */
    unsigned int cache_size_p;
    /*
     * Number of bits in the nested call counter is 2call_counter_size_p.
     * Minimum number of entries is 2, so a value of 0 indicates that
     * there is no implicit return call counter implemented.
     */
    unsigned int call_counter_size_p;
    /* Width of the ctype bus */
    unsigned int ctype_width_p;
    /* Width of context bus */
    unsigned int context_width_p;
    /* Width of time bus */
    unsigned int time_width_p;
    /* Width of exception cause bus */
    unsigned int ecause_width_p;
    /* Number of bits of exception cause to match using multiple choice */
    unsigned int ecause_choice_p;
    /*
     * Width of the subformat field in format 0 te_inst packets
     * (see Section 7.8.1).
     */
    unsigned int f0s_width_p;
    /* 0 or 1 Filtering on context supported when 1 */
    unsigned int filter_context_p;
    /* 0 or 1 Filtering on time supported when 1 */
    unsigned int filter_time_p;
    /*
     * Filtering on exception cause or interrupt supported when non_zero.
     * Number of nested exceptions supported is 2filter_excint_p
     */
    unsigned int filter_excint_p;
    /* 0 or 1 Filtering on privilege supported when 1 */
    unsigned int filter_privilege_p;
    /*
     * 0 or 1 Filtering on trap value supported when 1 (provided
     * filter_excint_p is non-zero)
     */
    unsigned int filter_tval_p;
    /*
     * LSB of instruction address bus to trace. 1 is compressed
     * instructions are supported, 2 otherwise
     */
    unsigned int iaddress_lsb_p;
    /* Width of instruction address bus. This is the same as DXLEN */
    unsigned int iaddress_width_p;
    /* Width of the iretire bus */
    unsigned int iretire_width_p;
    /* Width of the ilastsize bus */
    unsigned int ilastsize_width_p;
    /* Width of the itype bus */
    unsigned int itype_width_p;
    /* 0 or 1 Exclude context from te_inst packets if 1 */
    unsigned int nocontext_p;
    /* 0 or 1 Exclude time from te_inst packets if 1 */
    unsigned int notime_p;
    /* Width of privilege bus */
    unsigned int privilege_width_p;
    /* Maximum number of instructions that can be retired per block */
    unsigned int retires_p;
    /*
     * Number of entries in the return address stack is 2return_stack_size_p.
     * Minimum number of entries is 2, so a value of 0 indicates that there
     * is no implicit return stack implemented.
     */
    unsigned int return_stack_size_p;
    /* 0 or 1 sijump is used to identify sequentially inferable jumps */
    unsigned int sijump_p;
    /* Width of implementation-defined input bus */
    unsigned int impdef_width_p;
};

void rv_etrace_create_csv_file(void);

#endif
