/*------------------------------------------------------*/
/* Start code that enables and handles IRQs 		*/
/*------------------------------------------------------*/
/* Code modified specifically for this testbench	*/
/*------------------------------------------------------*/
/* Memory addresses reserved:				*/
/* 0000-0007  : Interrupt routine (part 1)		*/
/* 000a       :	Step count (0, 1, 2)			*/	
/* 000b       :	Sample time sub-count (4 bits)		*/	
/* 000c-000f  :	Last data position			*/
/* 0010-0013  :	Current timestamp (ms, approximately)	*/
/* 0014-0017  :	"start" value				*/
/* 0018-002f  : Save/restore register values		*/
/* 0030-start :	Interrupt routine (part 2)		*/
/* start-05fe :	Audio sample ring buffer		*/
/* 05fe-07fe  : Copy of ring buffer 1st 0x200 samples	*/
/*------------------------------------------------------*/

#undef ENABLE_FASTIRQ

#include "../custom_ops.S"

.section .text
.global Reset_Handler
.type Reset_Handler, %function
.global irq

Reset_Handler:
	j start

/* Interrupt handler */

.balign 4
irq_vec:
	/* This part goes to address 0 and jumps to address 0x28 */
	j irq_call

.balign 4
irq_regs:
        // Reserve space for register save/restore values, step count,
	// and ring buffer pointer.  This space is needed in SRAM, not
	// the flash, but it's easier to reserve the space in flash and
	// copy everything to SRAM as a single block.
        .fill 11,4

.balign 4
irq_call:
	/* save registers at memory location 0x18 to 0x2f */

        sw gp,   0*4+0x18(zero)
        sw t0,   1*4+0x18(zero)
        sw t1,   2*4+0x18(zero)
        sw t2,   3*4+0x18(zero)
        sw t4,   4*4+0x18(zero)
        sw t5,   5*4+0x18(zero)

	// Register use:
	// t0 :  SPI base address
	// t1 :  Step value
	// t2 :  Ring buffer position
	// t4 :  SPI read value
	// t5 :  Temporary values

	// Reset timer for a 16kHz count;  at 8MHz crystal external, 64MHz internal
	// PLL clock, that's a count of 4000 clock cycles.  Subtract of number of
	// clocks to reach this point after the interrupt, and divide by 3 = 1300
	// = 0x514
	
	li t5, 0x00000514
	picorv32_timer_insn(t5, t5)

	// This method works in three steps.  The step count is kept at address 10
	lb   t1, 10(zero)

	/* Increment the step count */
	addi t1, t1, 1

	/* If the step count is 3, set back to zero */

	addi t5, t1, -3 
	bnez t5, irq_1

	sb   zero, 10(zero)

	/* Update the timestamp sub-count */

	lb   t2, 11(zero)

	/* Increment the timestamp sub-count */
	addi t2, t2, 1

	/* If the sub-count is 16, set back to zero */
	addi t5, t2, -16
	bnez t5, irq_0
	li   t2, 0x0

	/* Increment the ms timestamp count */
	lw   t5, 16(zero)
	addi t5, t5, 1
	sw   t5, 16(zero)

irq_0:
	/* Update the timestamp sub-count */
	sb   t2, 11(zero)
	j    irq_2

irq_1:
	/* Write back updated step count */
	sb   t1, 10(zero)

irq_2:
	/* Get next ring buffer position and store in t2 */
	lw   t2, 12(zero)
	addi t2, t2, 2

	/* Wrap ring buffer after 0x05fe back to start */
	li  t5, 0x600
	sub  t5, t2, t5
        bnez t5, irq_3
	/* Note:  Want the SRAM address, which is the low part	*/
	/* of irq_end (flash address), shifted back by 4 bytes.	*/
	li   t2, %lo(irq_end)
	addi t2, t2, -4
irq_3:

	/* Set SPI base address */
	/* (0x030000b8 = config, 0x030000bc = data) */
	li   t0, 0x030000b8

	/* Check if the step count is 3 */
	addi t5, t1, -3
	bnez t5, irq_5

	/* Read SPI byte */
	lb   t4, 4(t0)

	/* Store in next ring buffer position lower 8 bits */
	sb   t4, 0(t2)

	/* If ring buffer position is in the lower 0x200 samples, then	*/
	/* copy the value to the section between 0x600 and 0x800.	*/
	li   t5, 0x200
	sub  t5, t2, t5
	/* Again, looking for the SRAM position equivalent to irq_end */
	li   t4, %lo(irq_end)
	addi t4, t4, -4
	blt  t5, t4, irq_4

	sub  t5, t2, t4
	addi t5, t5, 0x600
	
	lb   t4, 0(t2)
	sb   t4, 0(t5)
	lb   t4, 1(t2)
	sb   t4, 1(t5)

irq_4:
	/* Clear stream mode (clears CSB) */
	li	t5, 0x2000
	sh	t5, 0(t0)

	/* Update the ring buffer position */
	sw   t2, 12(zero)

	j irq_done
	
irq_5:
	/* Check if the step count is 2 */
	addi t5, t1, -2
	bnez t5, irq_6

	/* Step 2 only: */
	/* Read SPI byte */
	lb   t4, 4(t0)

	/* Store in next ring buffer position upper 8 bits */
	sb   t4, 1(t2)

irq_6:
	/* Check if the step count is 1 */
	addi t5, t1, -1
	bnez t5, irq_7

	/* Configure SPI for stream mode (sets CSB until cleared) */
	li	t5, 0x3014
	sh	t5, 0(t0)

irq_7:
	// Steps 1 and 2:
	/* Initiate SPI transfer.  Write zero to reg_spi_data */
	sw	zero, 4(t0)

irq_done:
        /* restore registers */

        lw gp,   0*4+0x18(zero)
        lw t0,   1*4+0x18(zero)
        lw t1,   2*4+0x18(zero)
        lw t2,   3*4+0x18(zero)
        lw t4,   4*4+0x18(zero)
        lw t5,   5*4+0x18(zero)

        picorv32_retirq_insn()

.balign 4
irq_end:

/* Main program */

.balign 4
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

# copy data section
la a0, _sidata
la a1, _sdata
la a2, _edata
bge a1, a2, end_init_data
loop_init_data:
lw a3, 0(a0)
sw a3, 0(a1)
addi a0, a0, 4
addi a1, a1, 4
blt a1, a2, loop_init_data
end_init_data:

# zero-init bss section
la a0, _sbss
la a1, _ebss
bge a0, a1, end_init_bss
loop_init_bss:
sw zero, 0(a0)
addi a0, a0, 4
blt a0, a1, loop_init_bss
end_init_bss:

# Copy the interrupt handler into SRAM starting at position 0.
lui  t0, %hi(irq_vec)
addi t0, t0, %lo(irq_vec)
lui  t1, %hi(irq_end)
addi t1, t1, %lo(irq_end)
li   t2, 0x0

next_copy:
lw   t4, 0(t0)
sw   t4, 0(t2)
addi t2, t2, 4
addi t0, t0, 4
blt  t0, t1, next_copy

# Set step value (0) in memory position 0x000a
sb   zero, 0x0a(zero)

# Set timestamp sub-count (0) in memory position 0x000b
sb   zero, 0x0b(zero)

# Set timestamp (0) in memory position 0x0010
sw   zero, 0x10(zero)

# Set the value of "start" (currently in t2) in memory position 0x0014
# This does not change, so a C program can always find the buffer start address.
sw   t2, 0x14(zero)

# Set ring buffer address +1 in memory position 0x000c
# Store a "silent" sample value (0x800) at the ring buffer start address. 
li   t5, 0x800
sw   t5, 0(t2)
addi t2, t2, 2
sw   t2, 12(zero)

# Enable timer IRQ only (0x01).
# NOTE: bus error should be masked so that it raises trap instead of interrupt.
li t4, 0xfffe
picorv32_maskirq_insn(t4, t4)

# Set the picorv32 32-bit counter/timer to trigger the first interrupt.
# After that it's the responsibility of the irq() routine to deal with it.
# (see above)  The first interrupt is not timed and is only needed to
# trigger the first interrupt and start the SPI data sampling cycle.

li t4, 0x200
picorv32_timer_insn(t4, t4)

# call main
call main
loop:
j loop

.global flashio_worker_begin
.global flashio_worker_end

.balign 4

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
li   t5, 32
lw   t2, 0(a0)
flashio_worker_LY:
li   t6, 8
flashio_worker_L2:
srli t4, t2, 31
sb   t4, 0(t0)
ori  t4, t4, 0x10
sb   t4, 0(t0)
lbu  t4, 0(t0)
andi t4, t4, 2
srli t4, t4, 1
slli t2, t2, 1
or   t2, t2, t4
addi t5, t5, -1
bnez t5, flashio_worker_LX
sw   t2, 0(a0)
addi a0, a0, 4
lw   t2, 0(a0)
flashio_worker_LX:
addi t6, t6, -1
bnez t6, flashio_worker_L2
addi a1, a1, -1
bnez a1, flashio_worker_LY
beqz t5, flashio_worker_L3
sw   t2, 0(a0)

flashio_worker_L3:
# Back to MEMIO mode
li   t1, 0x80
sb   t1, 3(t0)

ret

.balign 4
flashio_worker_end:

