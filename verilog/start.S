/*----------------------------------------------*/
/* Start code that enables and handles IRQs 	*/
/* NOTE:  Requires defining function irq()	*/
/* in the C code.				*/
/*----------------------------------------------*/

#undef ENABLE_FASTIRQ

#include "custom_ops.S"

.section .text
.global irq

reset_vec:
	j start

/* Interrupt handler @ 0x0100004 */

.balign 4
irq_vec:
	/* save registers */

        sw gp,   0*4+0x200(zero)
        sw x1,   1*4+0x200(zero)
        sw x2,   2*4+0x200(zero)
        sw x3,   3*4+0x200(zero)
        sw x4,   4*4+0x200(zero)
        sw x5,   5*4+0x200(zero)
        sw x6,   6*4+0x200(zero)
        sw x7,   7*4+0x200(zero)
        sw x8,   8*4+0x200(zero)
        sw x9,   9*4+0x200(zero)
        sw x10, 10*4+0x200(zero)
        sw x11, 11*4+0x200(zero)
        sw x12, 12*4+0x200(zero)
        sw x13, 13*4+0x200(zero)
        sw x14, 14*4+0x200(zero)
        sw x15, 15*4+0x200(zero)
        sw x16, 16*4+0x200(zero)
        sw x17, 17*4+0x200(zero)
        sw x18, 18*4+0x200(zero)
        sw x19, 19*4+0x200(zero)
        sw x20, 20*4+0x200(zero)
        sw x21, 21*4+0x200(zero)
        sw x22, 22*4+0x200(zero)
        sw x23, 23*4+0x200(zero)
        sw x24, 24*4+0x200(zero)
        sw x25, 25*4+0x200(zero)
        sw x26, 26*4+0x200(zero)
        sw x27, 27*4+0x200(zero)
        sw x28, 28*4+0x200(zero)
        sw x29, 29*4+0x200(zero)
        sw x30, 30*4+0x200(zero)
        sw x31, 31*4+0x200(zero)

	/* call interrupt handler C function */

        lui sp, %hi(irq_stack)
        addi sp, sp, %lo(irq_stack)

        // arg0 = address of regs
        lui a0, %hi(irq_regs)
        addi a0, a0, %lo(irq_regs)

        // arg1 = interrupt type
        addi a1, tp, 0

        // call to C function
        jal ra, irq

        /* restore registers */

        lw gp,   0*4+0x200(zero)
        lw x1,   1*4+0x200(zero)
        lw x2,   2*4+0x200(zero)
        // do not restore x3 (gp)
        lw x4,   4*4+0x200(zero)
        lw x5,   5*4+0x200(zero)
        lw x6,   6*4+0x200(zero)
        lw x7,   7*4+0x200(zero)
        lw x8,   8*4+0x200(zero)
        lw x9,   9*4+0x200(zero)
        lw x10, 10*4+0x200(zero)
        lw x11, 11*4+0x200(zero)
        lw x12, 12*4+0x200(zero)
        lw x13, 13*4+0x200(zero)
        lw x14, 14*4+0x200(zero)
        lw x15, 15*4+0x200(zero)
        lw x16, 16*4+0x200(zero)
        lw x17, 17*4+0x200(zero)
        lw x18, 18*4+0x200(zero)
        lw x19, 19*4+0x200(zero)
        lw x20, 20*4+0x200(zero)
        lw x21, 21*4+0x200(zero)
        lw x22, 22*4+0x200(zero)
        lw x23, 23*4+0x200(zero)
        lw x24, 24*4+0x200(zero)
        lw x25, 25*4+0x200(zero)
        lw x26, 26*4+0x200(zero)
        lw x27, 27*4+0x200(zero)
        lw x28, 28*4+0x200(zero)
        lw x29, 29*4+0x200(zero)
        lw x30, 30*4+0x200(zero)
        lw x31, 31*4+0x200(zero)

        picorv32_retirq_insn()

irq_regs:
        // registers are saved to this memory region during interrupt handling
        // the program counter is saved as register 0
        .fill 32,4

        // stack for the interrupt handler
        .fill 128,4
irq_stack:

/* Main program */

start:

# zero-initialize register file
addi x1, zero, 0
# x2 (sp) is initialized by reset
addi x3, zero, 0
addi x4, zero, 0
addi x5, zero, 0
addi x6, zero, 0
addi x7, zero, 0
addi x8, zero, 0
addi x9, zero, 0
addi x10, zero, 0
addi x11, zero, 0
addi x12, zero, 0
addi x13, zero, 0
addi x14, zero, 0
addi x15, zero, 0
addi x16, zero, 0
addi x17, zero, 0
addi x18, zero, 0
addi x19, zero, 0
addi x20, zero, 0
addi x21, zero, 0
addi x22, zero, 0
addi x23, zero, 0
addi x24, zero, 0
addi x25, zero, 0
addi x26, zero, 0
addi x27, zero, 0
addi x28, zero, 0
addi x29, zero, 0
addi x30, zero, 0
addi x31, zero, 0

# zero initialize scratchpad memory
# setmemloop:
# sw zero, 0(x1)
# addi x1, x1, 4
# blt x1, sp, setmemloop

# Write these instructions to memory location zero and following:
# lui t4, 0x00100	= 00100eb7
# addi t4, t4, 4	= 0e91
# jalr t4, 0		= 000e80e7
#
# These three instructions jump to 0x00100004, which is the location
# of the interrupt handler.  For a fast interrupt handler, the whole
# handler should be moved into SRAM.  Note that "addi" takes a 12-bit
# signed value, so any number with the 12th bit set must be entered
# as its 2s complement.

lui t4, 0x00101
# addi t4, t4, 0xeb7
addi t4, t4, -329
sw t4, 0(zero)
lui t4, 0x80e71
# addi t4, t4, 0xe91
addi t4, t4, -367
sw t4, 4(zero)
lui t4, 0x00000
addi t4, t4, 0x00e
sw t4, 8(zero)

# Enable all IRQs. . .  Except the UART's, which won't shut up.
lui t4, 0x00000
addi t4, t4, 0x010
picorv32_waitirq_insn(zero)
picorv32_maskirq_insn(t4, t4)

# Set the picorv32 32-bit counter/timer to trigger the first interrupt.
# After that it's the responsibility of the irq() routine to deal with it.
# Set timer for a 16kHz count;  at 8MHz crystal external, 64MHz internal PLL
# clock, that's a count of 4000 clock cycles.

lui t4, 0x00001
addi t4, t4, -96
picorv32_timer_insn(t4, t4)

# call main
call main
loop:
j loop

.global flashio_worker_begin
.global flashio_worker_end

flashio_worker_begin:
# a0 ... data pointer
# a1 ... data length
# a2 ... optional WREN cmd (0 = disable)

# address of SPI ctrl reg
li   t0, 0x02000000

# Set CS high, IO0 is output
li   t1, 0x120
sh   t1, 0(t0)

# Enable Manual SPI Ctrl
sb   zero, 3(t0)

# Send optional WREN cmd
beqz a2, flashio_worker_L1
li   t5, 8
andi t2, a2, 0xff
flashio_worker_L4:
srli t4, t2, 7
sb   t4, 0(t0)
ori  t4, t4, 0x10
sb   t4, 0(t0)
slli t2, t2, 1
andi t2, t2, 0xff
addi t5, t5, -1
bnez t5, flashio_worker_L4
sb   t1, 0(t0)

# SPI transfer
flashio_worker_L1:
beqz a1, flashio_worker_L3
li   t5, 8
lbu  t2, 0(a0)
flashio_worker_L2:
srli t4, t2, 7
sb   t4, 0(t0)
ori  t4, t4, 0x10
sb   t4, 0(t0)
lbu  t4, 0(t0)
andi t4, t4, 2
srli t4, t4, 1
slli t2, t2, 1
or   t2, t2, t4
andi t2, t2, 0xff
addi t5, t5, -1
bnez t5, flashio_worker_L2
sb   t2, 0(a0)
addi a0, a0, 1
addi a1, a1, -1
j    flashio_worker_L1
flashio_worker_L3:

# Back to MEMIO mode
li   t1, 0x80
sb   t1, 3(t0)

ret
flashio_worker_end:

