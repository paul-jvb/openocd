/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * S32K1xx / KE15Z PROGRAM_PHRASE async flash loader, with optional
 * external-watchdog /ST strobe.
 *
 * Sister of contrib/loaders/flash/kinetis/kinetis_flash.s (which
 * targets Kinetis K-family PROGRAM_LONGWORD, 4-byte writes). This one
 * uses the FTFC PROGRAM_PHRASE command (0x07) -- 8 bytes per cycle.
 *
 * The optional strobe lets the loader toggle an arbitrary GPIO every
 * N inner-loop iterations to keep an external watchdog supervisor
 * (e.g., MAX1232) satisfied while the chip is held in our flash loop.
 * Without strobing, a supervisor with a TD shorter than the total
 * programming time will assert /RESET partway through, aborting the
 * algorithm. The strobe pin and rate are passed in via a config
 * struct in chip SRAM (address in r8); pass r8 = 0 to disable.
 *
 * Params at entry (set up by kinetis_write_block_phrase() in
 * kinetis.c via reg_params + target_run_flash_async_algorithm):
 *   r0 = flash destination address (incremented by 8 each phrase)
 *   r1 = phrase count (decremented each phrase, exits at 0)
 *   r2 = ring-buffer descriptor base (wp at [r2,0], rp at [r2,4],
 *        data area starts at [r2,8] and ends just before r3)
 *   r3 = working-area end (ring-buffer wrap point)
 *   r4 = FTFx peripheral base (= 0x40020000)
 *   r8 = strobe config struct base (0 = strobing disabled). Layout:
 *          [+0]   PCC_PORTx       (PCC clock-gate register)
 *          [+4]   PORTx_PCRn      (port pin mux register)
 *          [+8]   PTx_PDDR        (data direction register)
 *          [+12]  PTx_PSOR        (set-output register, used for init)
 *          [+16]  PTx_PTOR        (toggle-output register)
 *          [+20]  mask            (bit mask for the pin, e.g. 1<<3)
 *          [+24]  count_reload    (toggle every N iterations)
 *          [+28]  count_remain    (loader scratch -- init to reload)
 *
 * Ring-buffer protocol matches target_run_flash_async_algorithm():
 *  - Host pushes 8 bytes per iteration into the ring at wp, advances wp.
 *  - Algorithm spins until rp != wp, programs the phrase, advances rp.
 *  - Host treats wp == 0 as a stop request (we honour it here too).
 *  - On error we set rp = 0 to signal the host.
 */

	.text
	.cpu cortex-m0plus
	.code 16
	.thumb_func

	.align	2

	/* FTFC register offsets from FTFx base (0x40020000) */
	.equ FTFx_FSTAT,  0
	.equ FTFx_FCCOB3, 4
	.equ FTFx_FCCOB0, 7
	.equ FTFx_FCCOB7, 8
	.equ FTFx_FCCOBB, 12

	.equ CMD_PHRASEPROG, 7
	.equ FSTAT_CCIF,     0x80
	.equ FSTAT_ERR_MASK, 0x70   /* ACCERR | FPVIOL | RDCOLERR */

	/* Offsets of fields in the strobe config struct (pointed-to by r8) */
	.equ ST_PCC,           0
	.equ ST_PCR,           4
	.equ ST_PDDR,          8
	.equ ST_PSOR,          12
	.equ ST_PTOR,          16
	.equ ST_MASK,          20
	.equ ST_COUNT_RELOAD,  24
	.equ ST_COUNT_REMAIN,  28

	/* One-time /ST init: configure the pin as a GPIO output and
	 * drive it HIGH. Skipped entirely if r8 == 0 (strobing disabled).
	 * Uses r5, r6, r7 as scratch; preserves r0-r4 and r8. */
init_st:
	mov	r5, r8			/* r5 = config base */
	cmp	r5, #0
	beq	init_done

	/* PCC: enable clock gate for the port (CGC bit, 0x40000000) */
	ldr	r6, [r5, #ST_PCC]
	ldr	r7, =0x40000000
	str	r7, [r6]

	/* PORTx_PCRn: MUX = 001 (GPIO ALT1, value 0x100) */
	ldr	r6, [r5, #ST_PCR]
	ldr	r7, =0x00000100
	str	r7, [r6]

	/* PTx_PDDR |= mask  (set pin as output, read-modify-write so we
	 * don't clobber other port-direction bits) */
	ldr	r6, [r5, #ST_PDDR]
	ldr	r7, [r6]
	ldr	r5, [r5, #ST_MASK]	/* r5 := mask (clobbers config base) */
	orr	r7, r5
	mov	r5, r8			/* restore r5 = config base */
	ldr	r6, [r5, #ST_PDDR]
	str	r7, [r6]

	/* PTx_PSOR = mask  (drive pin HIGH initially, so the first
	 * toggle is H->L) */
	ldr	r6, [r5, #ST_PSOR]
	ldr	r7, [r5, #ST_MASK]
	str	r7, [r6]

	/* count_remain := count_reload */
	ldr	r7, [r5, #ST_COUNT_RELOAD]
	str	r7, [r5, #ST_COUNT_REMAIN]

init_done:

wait_fifo:
	ldr	r6, [r2, #0]		/* read wp */
	cmp	r6, #0			/* stop if host set wp == 0 */
	beq	exit

	ldr	r5, [r2, #4]		/* read rp */
	cmp	r5, r6			/* spin until phrase available */
	beq	wait_fifo

	/* Inline /ST strobe (if enabled). Clobbers only r6, r7;
	 * preserves r5 (rp) for the FCCOB writes below. */
	mov	r6, r8
	cmp	r6, #0
	beq	skip_strobe_wf
	ldr	r7, [r6, #ST_COUNT_REMAIN]
	sub	r7, #1
	str	r7, [r6, #ST_COUNT_REMAIN]
	bne	skip_strobe_wf
	/* count hit 0 -> toggle pin and reload */
	ldr	r7, [r6, #ST_COUNT_RELOAD]
	str	r7, [r6, #ST_COUNT_REMAIN]
	ldr	r7, [r6, #ST_PTOR]
	ldr	r6, [r6, #ST_MASK]
	str	r6, [r7]		/* *PTOR = mask -> toggle */
skip_strobe_wf:

	/* Set up FCCOB for PROGRAM_PHRASE */
	str	r0, [r4, #FTFx_FCCOB3]	/* address (r4 byte 3 will be overwritten by cmd) */
	movs	r7, #CMD_PHRASEPROG
	strb	r7, [r4, #FTFx_FCCOB0]

	ldr	r7, [r5]
	str	r7, [r4, #FTFx_FCCOB7]

	ldr	r7, [r5, #4]
	str	r7, [r4, #FTFx_FCCOBB]

	/* Launch */
	movs	r7, #FSTAT_CCIF
	strb	r7, [r4, #FTFx_FSTAT]

	/* Advance rp by 8 with ring-buffer wrap */
	add	r5, #8
	cmp	r5, r3
	bcc	no_wrap
	mov	r5, r2
	add	r5, #8
no_wrap:
	str	r5, [r2, #4]

wait_ccif:
	ldr	r6, [r2, #0]		/* re-check wp (stop request) */
	cmp	r6, #0
	beq	exit

	ldrb	r6, [r4, #FTFx_FSTAT]
	movs	r7, #FSTAT_CCIF
	tst	r6, r7
	bne	ccif_done

	/* CCIF still 0 -> FTFC busy. Inline strobe + keep waiting. */
	mov	r6, r8
	cmp	r6, #0
	beq	wait_ccif
	ldr	r7, [r6, #ST_COUNT_REMAIN]
	sub	r7, #1
	str	r7, [r6, #ST_COUNT_REMAIN]
	bne	wait_ccif
	ldr	r7, [r6, #ST_COUNT_RELOAD]
	str	r7, [r6, #ST_COUNT_REMAIN]
	ldr	r7, [r6, #ST_PTOR]
	ldr	r6, [r6, #ST_MASK]
	str	r6, [r7]
	b	wait_ccif

ccif_done:
	/* CCIF set: check error bits */
	movs	r7, #FSTAT_ERR_MASK
	tst	r6, r7
	bne	error

	add	r0, #8			/* flash address += 8 (only on success) */
	sub	r1, #1			/* phrase_count-- */
	bne	wait_fifo
	b	exit

error:
	movs	r5, #0
	str	r5, [r2, #4]		/* rp = 0 -> signal error to host */

exit:
	bkpt	#0

	.ltorg				/* emit literal pool here */
