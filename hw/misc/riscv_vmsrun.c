/*
  * riscv_rpmi.c
  * RPMI transport IO handling routines.
  *
  * Copyright (c) 2024 Ventana Micro Systems Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.

  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.

  * You should have received a copy of the GNU General Public License along
  * with this program; if not, see <http://www.gnu.org/licenses/>.
  */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/misc/riscv_rpmi.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"
#include "hw/qdev-properties.h"
#include "system/runstate.h"
#include "target/riscv/cpu.h"
#include "vmsrun.h"
#include "vmsrun_env.h"
#include "librpmi.h"
#include "librpmi_env.h"

/*************** Common definitions and defines and functions ***************/
struct vmsrun_config cfg_vmsrun;

#define PLAT_INFO                       "veyron_v2_1.0"
#define PLAT_INFO_LEN                   strlen(PLAT_INFO)

int __printf(1, 2) rpmi_env_printf(const char *format, ...)
{
	int res;
	va_list args;
	va_start(args, format);
	res = vprintf(format, args);
	va_end(args);
	return res;
}

int __printf(3, 4) vmsrun_env_snprintf(char *out, rpmi_uint32_t out_len, const char *format, ...)
{
	int res;
	va_list args;
	va_start(args, format);
	res = vsnprintf(out, out_len, format, args);
	va_end(args);
	return res;
}

void vmsrun_env_nsdelay(rpmi_uint64_t nsec)
{
	return;
}

void *rpmi_env_zalloc(rpmi_size_t size)
{
	return calloc(1, size);
}

void rpmi_env_free(void *ptr)
{
	return free(ptr);
}

void rpmi_env_writel(rpmi_uint64_t addr, rpmi_uint32_t val)
{
	cpu_physical_memory_write(addr, &val, 4);
}

/******************** RPMI Shared Memory Platform defines *****************/

void handle_rpmi_event(void)
{
    vmsrun_run();
    return;
}

static uint64_t riscv_rpmi_read(void *opaque, hwaddr offset, unsigned int size)
{
    struct RiscvRpmiState *s = opaque;
    if ((size != 4) || (offset != 0)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "riscv_rpmi_read: Invalid read access to "
                      "addr %" HWADDR_PRIx ", size: %x\n",
                      offset, size);
        return 0;

    } else {
        return s->doorbell;
    }
}

static void riscv_rpmi_write(void *opaque, hwaddr offset,
                uint64_t val64, unsigned int size)
{
    struct RiscvRpmiState *s = opaque;
    if ((size != 4) || (offset != 0)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "riscv_rpmi_write: Invalid write access to "
                      "addr %" HWADDR_PRIx ", size: %x\n",
                      offset, size);
        return;
    }

    s->doorbell = val64;
    if (val64 == 1) {
        handle_rpmi_event();

        /* clear the doorbell register */
        s->doorbell = 0;
    }
}

static const MemoryRegionOps riscv_rpmi_ops = {
    .read = riscv_rpmi_read,
    .write = riscv_rpmi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void riscv_rpmi_realize(DeviceState *dev, Error **errp)
{
    RiscvRpmiState *rpmi = RISCV_RISCV_RPMI(dev);

    memory_region_init_io(&rpmi->mmio, OBJECT(dev), &riscv_rpmi_ops, rpmi,
                           TYPE_RISCV_RPMI, RPMI_DBREG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &rpmi->mmio);
 }

static void riscv_rpmi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = riscv_rpmi_realize;
    //device_class_set_props(dc, riscv_rpmi_properties);
}

static const TypeInfo riscv_rpmi_info = {
    .name          = TYPE_RISCV_RPMI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvRpmiState),
    .class_init    = riscv_rpmi_class_init,
};

static enum rpmi_error shmem_qemu_read(void *priv, rpmi_uint64_t addr,
                                       void *in, rpmi_uint32_t len)
{
    cpu_physical_memory_read(addr, in, len);
    return RPMI_SUCCESS;
}

static enum rpmi_error shmem_qemu_write(void *priv, rpmi_uint64_t addr,
                                        const void *out, rpmi_uint32_t len)
{
    cpu_physical_memory_write(addr, out, len);
    return RPMI_SUCCESS;
}

static enum rpmi_error shmem_qemu_fill(void *priv, rpmi_uint64_t addr,
                                       char ch, rpmi_uint32_t len)
{
    while (len > 0) {
        shmem_qemu_write(priv, addr, &ch, 1);
        len--;
        addr++;
    }

    return RPMI_SUCCESS;
}

struct rpmi_shmem_platform_ops rpmi_shmem_qemu_ops = {
    .read = shmem_qemu_read,
    .write = shmem_qemu_write,
    .fill = shmem_qemu_fill,
};


/************************ RPMI HSM Platform defines *************************/

#define RPMI_MAX_HARTS 128

/* Hart states array */
uint32_t hart_states[RPMI_MAX_HARTS] = {
    [0 ... (RPMI_MAX_HARTS - 1)] = RPMI_HART_HW_STATE_STARTED
} ;

static enum rpmi_hart_hw_state hart_get_hw_state(void *priv, rpmi_uint32_t hart_index)
{
    return hart_states[hart_index];
}

static enum rpmi_error hart_start_prepare(void *priv, rpmi_uint32_t hart_index,
                                          rpmi_uint64_t start_addr)
{
    CPUState *cpu = cpu_by_arch_id(hart_index);

    cpu->hold_stop = false;
    cpu_resume(cpu);
    hart_states[hart_index] = RPMI_HART_HW_STATE_STARTED;

    return RPMI_SUCCESS;
}

static void hart_start_finalize(void *priv, rpmi_uint32_t hart_index,
                                rpmi_uint64_t start_addr)
{
    return;
}

static bool execute_rpmi_hsm_stop(void *env)
{
    CPUState *cs = env_cpu(env);

    riscv_set_wfi_cb(env,  NULL);
    cs->stop = true;
    cs->hold_stop = true;
    qemu_cpu_kick(cs);
    hart_states[cs->cpu_index] = RPMI_HART_HW_STATE_STOPPED;

    return true;
}

/** Perpare a hart to stop (optional) */
static enum rpmi_error hart_stop_prepare(void *priv, rpmi_uint32_t hart_index)
{
    CPUState *cpu = cpu_by_arch_id(hart_index);
    CPURISCVState *env = &RISCV_CPU(cpu)->env;

    assert(env);
    riscv_set_wfi_cb(env, execute_rpmi_hsm_stop);
    hart_states[hart_index] = RPMI_HART_HW_STATE_STOPPED;

    return RPMI_SUCCESS;
}

/** Finalize hart stop (optional) */
static void hart_stop_finalize(void *priv, rpmi_uint32_t hart_index)
{
    return;
}

struct rpmi_hsm_platform_ops hsm_ops = {
    .hart_get_hw_state = hart_get_hw_state,
    .hart_start_prepare = hart_start_prepare,
    .hart_start_finalize = hart_start_finalize,
    .hart_stop_prepare = hart_stop_prepare,
    .hart_stop_finalize = hart_stop_finalize,
};

/********************* RPMI System MSI Platform defines **********************/

#define RPMI_SYS_MSI_SHUTDOWN_INDEX	    0
#define RPMI_SYS_MSI_REBOOT_INDEX	    1
#define RPMI_SYS_MSI_SUSPEND_INDEX	    2
#define RPMI_SYS_MSI_P2A_DB_INDEX	    3
#define RPMI_SYS_NUM_MSI		        4

static rpmi_bool_t sysmsi_validate_msi_addr(void *priv, rpmi_uint64_t msi_addr)
{
    return true;
}

static struct rpmi_sysmsi_platform_ops sysmsi_ops = {
    .validate_msi_addr = sysmsi_validate_msi_addr,
};

void riscv_rpmi_inject_sysmsi(uint32_t sys_msi_index)
{
    vmsrun_rpmi_inject_sysmsi(sys_msi_index);
}

/********************* RPMI System Reset Platform defines ********************/

rpmi_uint32_t rpmi_reset_types[2] = {
    RPMI_SYSRST_TYPE_SHUTDOWN,
    RPMI_SYSRST_TYPE_COLD_REBOOT
};

static void rpmi_do_system_reset(void *priv, rpmi_uint32_t reset_type)
{
    if (reset_type == RPMI_SYSRST_TYPE_WARM_REBOOT ||
        reset_type == RPMI_SYSRST_TYPE_COLD_REBOOT) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: rebooting..\n",  __func__);
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    } else if (reset_type == RPMI_SYSRST_TYPE_SHUTDOWN) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Shutting down..\n", __func__);
        exit(0);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid reset service Id..\n", __func__);
        return;
    }

    return;
}

struct rpmi_sysreset_platform_ops rpmi_reset_ops = {
    .do_system_reset = rpmi_do_system_reset
};

/********************* RPMI System Suspend Platform defines ********************/

#define RPMI_SYSSUSP_SHUTDOWN           0
#define RPMI_SYSSUSP_COLD_SUSPEND       1
#define RPMI_SYSSUSP_SUSPEND            2

struct rpmi_system_suspend_type rpmi_suspend_types[1] = {
    { .type = RPMI_SYSSUSP_SHUTDOWN, .attr = 0 },
};

static bool execute_rpmi_suspend(void *env)
{
    riscv_set_wfi_cb(env,  NULL);
    qemu_system_suspend_request();
    return true;
}

struct system_suspend_wakeup_notifier {
    void *priv_data;
    Notifier wakeup;
};

static struct system_suspend_wakeup_notifier *system_suspend_wakeup;

static void system_suspend_wakeup_notify(Notifier *notifier, void *data)
{
    vmsrun_rpmi_process_suspend_event(system_suspend_wakeup->priv_data);
}

static enum rpmi_error system_suspend_prepare(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    CPUState *cpu;
    CPURISCVState *env;

    cpu = cpu_by_arch_id(hart_index);
    env = cpu ? &RISCV_CPU(cpu)->env : NULL;

    if (!system_suspend_wakeup) {
        system_suspend_wakeup = rpmi_env_zalloc(sizeof(*system_suspend_wakeup));
        system_suspend_wakeup->priv_data = priv;
        system_suspend_wakeup->wakeup.notify = system_suspend_wakeup_notify;
        qemu_register_wakeup_notifier(&system_suspend_wakeup->wakeup);
    } else {
        g_assert(system_suspend_wakeup->priv_data == priv);
    }

    qemu_register_wakeup_support();

    riscv_set_wfi_cb(env,  execute_rpmi_suspend);

    return RPMI_SUCCESS;
}

static rpmi_bool_t system_suspend_ready(void *priv, rpmi_uint32_t hart_index)
{
    return true;
}

static void system_suspend_finalize(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    return;
}

static rpmi_bool_t system_suspend_can_resume(void *priv, rpmi_uint32_t hart_index)
{
    return true;
}

static enum rpmi_error system_suspend_resume(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    return RPMI_SUCCESS;
}

struct rpmi_syssusp_platform_ops rpmi_suspend_ops = {
    .system_suspend_prepare = system_suspend_prepare,
    .system_suspend_ready = system_suspend_ready,
    .system_suspend_finalize = system_suspend_finalize,
    .system_suspend_can_resume = system_suspend_can_resume,
    .system_suspend_resume = system_suspend_resume
};

/*********************** RPMI CPPC Platform defines *************************/

target_ulong helper_csrr(CPURISCVState *env, int csr);

static enum rpmi_error cppc_get_reg(void *priv, rpmi_uint32_t reg_id,
                             rpmi_uint32_t hart_id, rpmi_uint64_t *val)
{
    CPUState *cpu = cpu_by_arch_id(hart_id);
    CPURISCVState *env = &RISCV_CPU(cpu)->env;
    uint64_t mcycle = 50;

    mcycle = helper_csrr(env, CSR_TIME);

    switch (reg_id) {
    case RPMI_CPPC_DELIVERED_PERF_COUNTER:
        /* dummy delivered performance */
        *val = mcycle * (4);
        break;

    case RPMI_CPPC_REFERENCE_PERF_COUNTER:
        *val = mcycle;
        break;
    }

    return RPMI_SUCCESS;
}

static enum rpmi_error cppc_set_reg(void *priv, rpmi_uint32_t reg_id,
                             rpmi_uint32_t hart_id, rpmi_uint64_t val)
{
    return RPMI_SUCCESS;
}

static enum rpmi_error cppc_update_perf(void *priv, rpmi_uint32_t desired_perf,
                                 rpmi_uint32_t hart_index)
{
    return RPMI_SUCCESS;
}

static enum rpmi_error cppc_get_current_freq(void *priv, rpmi_uint32_t hart_index,
                                      rpmi_uint64_t *cur_freq)
{
    /* dummy value */
    *cur_freq = 0xdeadbeeffeedbead;
    return 0;
}

struct rpmi_cppc_platform_ops cppc_ops = {
    .cppc_get_reg = cppc_get_reg,
    .cppc_set_reg = cppc_set_reg,
    .cppc_update_perf = cppc_update_perf,
    .cppc_get_current_freq = cppc_get_current_freq,
};

/*********************** RPMI CPPC Platform defines *************************/

#define RPMI_CLOCK_COUNT        6

#define CLOCK_DIFF_NEGATIVE      0x100
#define CLOCK_DIFF_POSITIVE      0x100

struct platform_clocks_context {
    uint32_t current_state;
    uint64_t current_rate;
};

static uint64_t rate_linear[3] = {0x1111111122222222, 0xbbbbbbbbcccccccc,
                                    0x2222222222222222};
static uint64_t rate_discrete[6] = {0x1111111122222222, 0x2222222233333333,
                                    0x3333333344444444, 0x4444444455555555,
                                    0x5555555566666666, 0x6666666677777777};

static struct rpmi_clock_data clock_data[] = {
	[0] = {
		.name = "clock0",
		.parent_id = -1U,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[1] = {
		.name = "clock1",
		.parent_id = 0,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[2] = {
		.name = "clock2",
		.parent_id = 0,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},

	[3] = {
		.name = "clock3",
		.parent_id = 1,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},

	[4] = {
		.name = "clock4",
		.parent_id = 1,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[5] = {
		.name = "clock5",
		.parent_id = 4,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},
};

static struct platform_clocks_context platclks_ctx[RPMI_CLOCK_COUNT] = {
    [0] = {.current_rate = 0x1111111122222222, .current_state = RPMI_CLK_STATE_ENABLED},
    [1] = {.current_rate = 0xbbbbbbbbcccccccc, .current_state = RPMI_CLK_STATE_ENABLED},
    [2] = {.current_rate = 0x2222222233333333, .current_state = RPMI_CLK_STATE_ENABLED},
    [3] = {.current_rate = 0x3333333344444444, .current_state = RPMI_CLK_STATE_ENABLED},
    [4] = {.current_rate = 0x1111111122222222, .current_state = RPMI_CLK_STATE_ENABLED},
    [5] = {.current_rate = 0x5555555566666666, .current_state = RPMI_CLK_STATE_ENABLED},
};

static enum rpmi_error platform_set_state(void *priv,
                                   uint32_t clk_id,
                                   enum rpmi_clock_state state)
{

    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (state >= RPMI_CLK_STATE_MAX_IDX)
        return RPMI_ERR_INVALID_PARAM;

    platclks_ctx[clk_id].current_state = state;

    return RPMI_SUCCESS;
}

static enum rpmi_error platform_get_state_and_rate(void *priv,
                                            uint32_t clk_id,
                                            enum rpmi_clock_state *state,
                                            uint64_t *rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!state && !rate)
        return RPMI_ERR_INVALID_PARAM;

    if (state)
        *state = platclks_ctx[clk_id].current_state;

    if (rate)
        *rate = platclks_ctx[clk_id].current_rate;

    return RPMI_SUCCESS;

}

static enum rpmi_error platform_rate_change_match(void *priv,
                                   uint32_t clk_id,
                                   uint64_t rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return false;

    uint64_t current_rate = platclks_ctx[clk_id].current_rate;

    if (rate > current_rate && (rate - current_rate) > CLOCK_DIFF_POSITIVE)
        return true;

    if (rate < current_rate && (current_rate - rate) < CLOCK_DIFF_NEGATIVE)
        return true;

    return false;
}

static enum rpmi_error platform_set_rate(void *priv,
                                  uint32_t clk_id,
                                  enum rpmi_clock_rate_match match,
                                  uint64_t rate,
                                  rpmi_uint64_t *new_rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!platform_rate_change_match(NULL, clk_id, rate))
        return RPMI_ERR_ALREADY;

    switch(match) {
        case RPMI_CLK_RATE_MATCH_ROUND_UP:
            platclks_ctx[clk_id].current_rate = rate + 0x100;
            break;
        case RPMI_CLK_RATE_MATCH_ROUND_DOWN:
            platclks_ctx[clk_id].current_rate = rate - 0x100;
            break;
        case RPMI_CLK_RATE_MATCH_PLATFORM:
            platclks_ctx[clk_id].current_rate = rate + 0x200;
            break;
        default:
            return RPMI_ERR_INVALID_PARAM;
    };

    *new_rate = platclks_ctx[clk_id].current_rate;

    return RPMI_SUCCESS;
}

static enum rpmi_error platform_set_rate_recalc(void *priv,
                                   uint32_t clk_id,
                                   uint64_t parent_rate,
                                   uint64_t *new_rate)
{

    uint64_t rate = platclks_ctx[clk_id].current_rate;
    rate = (rate/parent_rate) * 1.5;
    platclks_ctx[clk_id].current_rate = rate;
    *new_rate = rate;
    return RPMI_SUCCESS;
}

const struct rpmi_clock_platform_ops clock_ops = {
    .set_rate = platform_set_rate,
    .set_state = platform_set_state,
    .set_rate_recalc = platform_set_rate_recalc,
    .get_state_and_rate = platform_get_state_and_rate,
    .rate_change_match = platform_rate_change_match,
};


/********************* Qemu - VMSRUN Interface ******************************/

static void vmsrun_init_config(hwaddr db_addr, hwaddr shm_addr, int shm_sz,
                               uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                               hwaddr fcm_addr, int fcm_sz, uint32_t base_hartid,
                               uint32_t hart_count, bool rpmi_context_system)
{
    if (rpmi_context_system) {
	    qemu_log_mask(LOG_GUEST_ERROR, "system context initialization\n");
        cfg_vmsrun.rpmi.system.sysmsi.num_msi = RPMI_SYS_NUM_MSI;
	    cfg_vmsrun.rpmi.system.sysmsi.p2a_msi_index = RPMI_SYS_MSI_P2A_DB_INDEX;
	    cfg_vmsrun.rpmi.system.sysmsi.ops = &sysmsi_ops;
	    cfg_vmsrun.rpmi.system.sysmsi.priv_data = NULL;

        cfg_vmsrun.rpmi.system.sysreset.reset_type_count =
                                sizeof(rpmi_reset_types)/sizeof(rpmi_uint32_t);
        cfg_vmsrun.rpmi.system.sysreset.reset_type_array = rpmi_reset_types;
        cfg_vmsrun.rpmi.system.sysreset.ops = &rpmi_reset_ops;
    
        cfg_vmsrun.rpmi.system.syssuspend.suspend_type_count =
                sizeof(rpmi_suspend_types)/sizeof(struct rpmi_system_suspend_type);
        cfg_vmsrun.rpmi.system.syssuspend.suspend_type_array = rpmi_suspend_types;
        cfg_vmsrun.rpmi.system.syssuspend.ops = &rpmi_suspend_ops;

        cfg_vmsrun.rpmi.system.clock.clock_count =
                            sizeof(clock_data)/sizeof(struct rpmi_clock_data);
        cfg_vmsrun.rpmi.system.clock.clock_data = clock_data;
        cfg_vmsrun.rpmi.system.clock.ops = &clock_ops;
        cfg_vmsrun.rpmi.system.clock.priv_data = NULL;
        
	    cfg_vmsrun.rpmi.system.trans.name = "system";
	    cfg_vmsrun.rpmi.system.trans.shmem_addr = shm_addr;
	    cfg_vmsrun.rpmi.system.trans.shmem_size = shm_sz;
        cfg_vmsrun.rpmi.system.trans.ops = &rpmi_shmem_qemu_ops;
	    cfg_vmsrun.rpmi.system.trans.plat_info = PLAT_INFO;
	    cfg_vmsrun.rpmi.system.trans.plat_info_len = PLAT_INFO_LEN;
	    cfg_vmsrun.rpmi.system.priv_data = NULL;
    }
    else {
	    qemu_log_mask(LOG_GUEST_ERROR, "chiplet context initialization\n");
        cfg_vmsrun.cluster_count = 1;

        cfg_vmsrun.cluster[0].i3c_pid = 0xdead1111;
	    cfg_vmsrun.cluster[0].hart_count = hart_count;
	    cfg_vmsrun.cluster[0].chipid = 0x0;
	    cfg_vmsrun.cluster[0].base_hartid = base_hartid;

	    cfg_vmsrun.rpmi.chiplet.trans.name = "chiplet";
	    cfg_vmsrun.rpmi.chiplet.trans.shmem_addr = shm_addr;
	    cfg_vmsrun.rpmi.chiplet.trans.shmem_size = shm_sz;
        cfg_vmsrun.rpmi.chiplet.trans.ops = &rpmi_shmem_qemu_ops;
	    cfg_vmsrun.rpmi.chiplet.trans.plat_info = PLAT_INFO;
	    cfg_vmsrun.rpmi.chiplet.trans.plat_info_len = PLAT_INFO_LEN;
	    cfg_vmsrun.rpmi.chiplet.priv_data = NULL;

        cfg_vmsrun.rpmi.chiplet.hsm.hart_suspend_count = 0;
	    cfg_vmsrun.rpmi.chiplet.hsm.ops = &hsm_ops;

        cfg_vmsrun.rpmi.chiplet.cppc.fc_shmem_addr = fcm_addr;
        cfg_vmsrun.rpmi.chiplet.cppc.fc_shmem_size = fcm_sz;
        cfg_vmsrun.rpmi.chiplet.cppc.fc_shmem_ops = &rpmi_shmem_qemu_ops;
        cfg_vmsrun.rpmi.chiplet.cppc.regs.highest_perf = 5,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.nominal_perf = 4,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.lowest_nonlinear_perf = 2,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.lowest_perf = 2,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.reference_perf = 1,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.lowest_freq = 40,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.nominal_freq = 80,
        cfg_vmsrun.rpmi.chiplet.cppc.regs.perf_limited = 0,
        cfg_vmsrun.rpmi.chiplet.cppc.ops = &cppc_ops;
        cfg_vmsrun.rpmi.chiplet.cppc.priv_data = NULL;
    }
}

DeviceState *riscv_rpmi_create(hwaddr db_addr, hwaddr shm_addr, int shm_sz,
                               uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                               hwaddr fcm_addr, int fcm_sz, uint32_t base_hartid,
                               uint32_t hart_count, bool rpmi_context_system,
                               MachineState *ms)
{
    DeviceState *dev = qdev_new(TYPE_RISCV_RPMI);
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *shm_mr = g_new0(MemoryRegion, 1);
    MemoryRegion *fcm_mr = g_new0(MemoryRegion, 1);
    char name[32];
    
	qemu_log_mask(LOG_GUEST_ERROR, "insinde riscv_rpmi_create\n");
    assert(!(db_addr & 0x3));
    assert(!(shm_addr & 0x3));

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, db_addr);

    sprintf(name, "shm@%lx", shm_addr);
    memory_region_init_ram(shm_mr, OBJECT(dev),
                           name, shm_sz, &error_fatal);
    memory_region_add_subregion(address_space_mem,
                                shm_addr, shm_mr);

    if (fcm_sz) {
        sprintf(name, "fcm@%lx", fcm_addr);
        memory_region_init_ram(fcm_mr, OBJECT(dev), name,
                               fcm_sz, &error_fatal);
        memory_region_add_subregion(address_space_mem,
                                    fcm_addr, fcm_mr);
    }
    
	qemu_log_mask(LOG_GUEST_ERROR, "calling vmsrun_init_config\n");
    vmsrun_init_config(db_addr, shm_addr, shm_sz,
                                a2preq_qsz, p2areq_qsz, fcm_addr, fcm_sz,
                                base_hartid, hart_count, rpmi_context_system);

    return dev;
}

void riscv_rpmi_init(void)
{
    int rc;
    rc = vmsrun_init(&cfg_vmsrun);
    if (rc) {
	    	qemu_log_mask(LOG_GUEST_ERROR, "vmsrun init failed");
	    	return;
	}
}

static void riscv_rpmi_register_types(void)
{
    type_register_static(&riscv_rpmi_info);
}

type_init(riscv_rpmi_register_types)
