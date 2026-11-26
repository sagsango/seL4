/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>
#include <machine/assembler.h>
#include <arch/api/syscall.h>
#include <arch/machine/hardware.h>
#include <arch/machine/registerset.h>

/* EXCEPTION 02: Hardware uses the vector base for the destination EL.
 * On entry, ELR_ELn holds the architectural return address, SPSR_ELn
 * holds the old PSTATE, and ESR_ELn describes a synchronous exception.
 * Hardware masks exceptions on entry and leaves x0-x30 for software
 * to save. FAR_ELn provides the fault address for applicable aborts.
 * With hypervisor support, this code runs at EL2. Otherwise it runs at EL1. */
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT

#define ELR     elr_el2
#define ESR     esr_el2
#define SPSR    spsr_el2
#define TPIDR   tpidr_el2

#else

#define ELR     elr_el1
#define ESR     esr_el1
#define SPSR    spsr_el1
#define TPIDR   tpidr_el1

#endif


.macro lsp_i _tmp
    mrs     \_tmp, TPIDR
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    bic     \_tmp, \_tmp, #0xfff
#endif
    mov     sp, \_tmp
.endm

.macro ventry label
.align 7
    b       \label
.endm

#ifdef CONFIG_DEBUG_BUILD
.macro ventry_invalid
.align 7
    /* compute vect_offset argument */
    adr     x0, .
    adr     x19, arm_vector_table
    sub     x0, x0, x19
    b       invalid_vector_entry
.endm
#else
.macro ventry_invalid
    ventry invalid_vector_entry
.endm
#endif

#ifdef CONFIG_AARCH64_SERROR_IGNORE
.macro ventry_serror
.align 7
    eret
.endm
#else
.macro ventry_serror
    ventry_invalid
.endm
#endif

.section .vectors, "ax"

/* The layout of this table are specified in Table D1-7, 'Vector offsets from
 * vector table base address' of ARM DDI 0487B.b ID092517.
 * Each vector entry has 128-byte alignment, and so supports 32 instructions.
 */
/* EXCEPTION 03: Four groups of four 128-byte entries start at offsets
 * 0x000 (current EL using SP0), 0x200 (current EL using SP_ELn),
 * 0x400 (lower EL in AArch64), and 0x600 (lower EL in AArch32).
 * Within each group the order is synchronous, IRQ, FIQ, SError.
 * At EL2 the lower AArch64 group can take traps from EL0 or guest EL1.
 * Architectural routing and the source state select the vector before
 * the kernel examines ESR. Unsupported entries go to the invalid handler. */
BEGIN_FUNC(arm_vector_table)
    ventry_invalid                         // Synchronous EL1t/EL2t
    ventry_invalid                         // IRQ EL1t/EL2t
    ventry_invalid                         // FIQ EL1t/EL2t
    ventry_invalid                         // SError EL1t/EL2t

    ventry  cur_el_sync                    // Current EL Synchronous (EL1/2)
    ventry  cur_el_irq                     // IRQ
    ventry_invalid                         // FIQ
    ventry_serror                          // SError

    /* [PAGE_FAULT_FLOW (02): An EL0 store with a missing translation takes the lower-EL
     * synchronous vector (VBAR_EL2 + 0x400 in this build).]
     */
    ventry  lower_el_sync                  // Synchronous 64-bit EL0/EL1
    ventry  lower_el_irq                   // IRQ 64-bit EL0/EL1
    ventry_invalid                         // FIQ 64-bit EL0/EL1
    ventry_serror                          // SError 64-bit EL0/EL1

    ventry_invalid                         // Synchronous 32-bit EL0/EL1
    ventry_invalid                         // IRQ 32-bit EL0/EL1
    ventry_invalid                         // FIQ 32-bit EL0/EL1
    ventry_invalid                         // SError 32-bit EL0/EL1
END_FUNC(arm_vector_table)

.section .vectors.text, "ax"

/* EXCEPTION 04: On lower-EL entry, the kernel SP points to the saved
 * register area for the running TCB. Save GPRs before using them as scratch,
 * then preserve SP_EL0, the return PC, PSTATE and TLS. After the stub records
 * FaultIP, lsp_i switches to the kernel stack before entering C.
 * PT_ELR_EL1 is a context layout name used for both EL1 and EL2 builds. */
.macro kernel_enter
    /* Storing thread's stack frame */
    stp     x0,  x1,  [sp, #16 * 0]
    stp     x2,  x3,  [sp, #16 * 1]
    stp     x4,  x5,  [sp, #16 * 2]
    stp     x6,  x7,  [sp, #16 * 3]
    stp     x8,  x9,  [sp, #16 * 4]
    stp     x10, x11, [sp, #16 * 5]
    stp     x12, x13, [sp, #16 * 6]
    stp     x14, x15, [sp, #16 * 7]
    stp     x16, x17, [sp, #16 * 8]
    stp     x18, x19, [sp, #16 * 9]
    stp     x20, x21, [sp, #16 * 10]
    stp     x22, x23, [sp, #16 * 11]
    stp     x24, x25, [sp, #16 * 12]
    stp     x26, x27, [sp, #16 * 13]
    stp     x28, x29, [sp, #16 * 14]

    /* Store thread's SPSR, LR, and SP */
    mrs     x21, sp_el0
    mrs     x22, ELR
    mrs     x23, SPSR
    stp     x30, x21, [sp, #PT_LR]
    stp     x22, x23, [sp, #PT_ELR_EL1]

    /* Save TLS before acquiring the kernel lock. A remote stall can
     * switch to idle and leave lock acquisition without reaching the
     * C entry hook. In particular, EL0 can write TPIDR_EL0 directly. */
    mrs     x21, tpidr_el0
    mrs     x22, tpidrro_el0
    stp     x21, x22, [sp, #PT_TPIDR_EL0]
.endm

/* Takes argument x0: vect_offset (in debug builds) */
BEGIN_FUNC(invalid_vector_entry)
    lsp_i   x19
#ifdef CONFIG_DEBUG_BUILD
    mrs     x1, ELR
    bl      c_handle_invalid_vector_entry
#endif
    b       halt
END_FUNC(invalid_vector_entry)

/* EXCEPTION 05: A same-EL synchronous abort is a kernel exception.
 * Debug builds can print data or instruction abort diagnostics before halt.
 * The lower-EL path can instead report a fault to a userspace endpoint. */
BEGIN_FUNC(cur_el_sync)
    lsp_i   x19
    /* Read esr and branch to respective labels */
    mrs     x25, ESR
    lsr     x24, x25, #ESR_EC_SHIFT
    cmp     x24, #ESR_EC_CEL_DABT
    b.eq    cur_el_da
    cmp     x24, #ESR_EC_CEL_IABT
    b.eq    cur_el_ia
    b       cur_el_inv

cur_el_da:
#ifdef CONFIG_DEBUG_BUILD
    mrs     x0, ELR
    bl      kernelDataAbort
#endif /* CONFIG_DEBUG_BUILD */
    b       halt

cur_el_ia:
#ifdef CONFIG_DEBUG_BUILD
    mrs     x0, ELR
    bl      kernelPrefetchAbort
#endif /* CONFIG_DEBUG_BUILD */
    b       halt

cur_el_inv:
    /* 0x200 corresponding to the offset in the vector table */
    mov     x0, #0x200
    b       invalid_vector_entry
END_FUNC(cur_el_sync)

/*
 * This is only called if ksCurThread is idle thread.
 *
 * No need to store the state of idle thread and simply call c_handle_interrupt to
 * activate ksCurThread when returning from interrupt as long as idle thread is stateless.
 */
BEGIN_FUNC(cur_el_irq)
    lsp_i   x19
    b       c_handle_interrupt
END_FUNC(cur_el_irq)

/* EXCEPTION 05: ESR.EC selects the handler. Instruction and data aborts
 * use the VM-fault path. SVC64 and, in a hypervisor build, HVC64 enter the
 * seL4 syscall ABI. Remaining classes go to the vCPU-fault dispatcher at
 * EL2 or the undefined/debug instruction handler in the EL1 build. */
BEGIN_FUNC(lower_el_sync)
    kernel_enter

    /* Read esr and branch to respective labels */
    mrs     x25, ESR
    lsr     x24, x25, #ESR_EC_SHIFT
    cmp     x24, #ESR_EC_LEL_DABT
    b.eq    lel_da
    cmp     x24, #ESR_EC_LEL_IABT
    b.eq    lel_ia
    cmp     x24, #ESR_EC_LEL_SVC64
    b.eq    lel_syscall
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
    cmp     x24, #ESR_EC_LEL_HVC64
    b.eq    lel_syscall
    mrs     x20, ELR
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19
    /* move the ESR as the input */
    mov     x0, x25
    b       c_handle_vcpu_fault
#else
    b       el0_user
#endif

/* [PAGE_FAULT_FLOW (03): lower_el_sync saved the GPRs and return state with kernel_enter.
 * ESR.EC selects lel_da. Copy ELR to FaultIP without subtracting 4,
 * switch stacks with lsp_i, then branch to c_handle_data_fault.]
 */
lel_da:
    mrs     x20, ELR
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19
    b       c_handle_data_fault

lel_ia:
    mrs     x20, ELR
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19
    b       c_handle_instruction_fault

/* EXCEPTION 06: SVC/HVC is a 4-byte instruction in AArch64. ELR holds
 * the following instruction. Save ELR - 4 as FaultIP for reporting or
 * restart, while preserving the following instruction as NextIP.
 * x7 selects the syscall. x0 carries the capability pointer for
 * IPC/invocations and x1 carries MessageInfo where applicable.
 * Selected IPC syscalls can try fastpath before falling back to C. */
lel_syscall:
    mrs     x20, ELR
    sub     x20, x20, #4
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19

#ifdef CONFIG_FASTPATH
    cmp     x7, #SYSCALL_CALL
    b.eq    c_handle_fastpath_call
#ifdef CONFIG_SIGNAL_FASTPATH
    cmp     x7, #SYSCALL_SEND
    b.eq    c_handle_fastpath_signal
#endif /* CONFIG_SIGNAL_FASTPATH */
    cmp     x7, #SYSCALL_REPLY_RECV
#ifdef CONFIG_KERNEL_MCS
    mov     x2, x6
#endif
    b.eq    c_handle_fastpath_reply_recv
#endif

    mov     x2, x7
    b       c_handle_syscall

el0_user:
    mrs     x20, ELR
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19
    b       c_handle_undefined_instruction
END_FUNC(lower_el_sync)

/* EXCEPTION 07: IRQ delivery is asynchronous. Save the interrupted
 * context and record ELR directly, without the syscall adjustment.
 * The interrupt handler can make another thread runnable before return. */

BEGIN_FUNC(lower_el_irq)
    kernel_enter
    mrs     x20, ELR
    str     x20, [sp, #PT_FaultIP]

    lsp_i   x19
    b       c_handle_interrupt
END_FUNC(lower_el_irq)
