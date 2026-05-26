/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * S32K1xx / KE15Z PROGRAM_CHECK margin-verify async loader.
 *
 * Per NXP S32K1xx RM section 31.4.12.3 ("Program Check"), the FTFC
 * PROGRAM_CHECK command (FCCOB0 = 0x02) re-reads one flash longword
 * and compares it against expected data at a configurable margin
 * level (FCCOB4 byte): 0x01 = USER margin (more stringent than the
 * normal read level — catches cells programmed close to threshold),
 * 0x02 = FACTORY margin (most stringent, intended for NXP factory
 * QA). Margin choice 0x00 is reserved for this command and gets
 * rejected by the chip with ACCERR.
 *
 * Loader structure mirrors kinetis_phrase.s: async ring-buffer
 * protocol, same r8-based strobe config so an external supervisor
 * IC (e.g. MAX1232 on a GPIO) stays satisfied throughout. Each loop
 * iteration consumes ONE longword (4 bytes) of expected data, issues
 * the FCCOB PROGRAM_CHECK at the configured margin, and reads back
 * FSTAT. On margin failure or any FSTAT error bit set, the loader
 * sets rp = 0 and exits with the failing flash address still in r0
 * — host can read r0 to know where the failure occurred.
 *
 * Params at entry (set up by kinetis_margin_check_block() in
 * kinetis.c via reg_params + target_run_flash_async_algorithm):
 *   r0 = flash address (longword-aligned, advances by 4 each iter)
 *   r1 = longword count (decrements each iter, exits at 0)
 *   r2 = ring-buffer descriptor base (wp at [r2,0], rp at [r2,4],
 *        data area starts at [r2,8] and ends just before r3)
 *   r3 = working-area end (ring-buffer wrap point)
 *   r4 = FTFx peripheral base (= 0x40020000)
 *   r8 = strobe config struct base (0 = strobing disabled). Same
 *        layout as in kinetis_phrase.s — see that file for details.
 *   r9 = margin choice byte placed into FCCOB4 (0x01 = USER,
 *        0x02 = FACTORY).
 */

	.text
	.cpu cortex-m0plus
	.code 16
	.thumb_func

	.align	2

	.equ FTFx_FSTAT,  0
	.equ FTFx_FCCOB3, 4
	.equ FTFx_FCCOB0, 7
	.equ FTFx_FCCOB7, 8
	.equ FTFx_FCCOBB, 12

	.equ CMD_PROGCHECK,  2
	.equ FSTAT_CCIF,     0x80
	.equ FSTAT_ERR_MASK, 0x70   /* ACCERR | FPVIOL | RDCOLERR */

	/* Strobe config struct offsets (same layout as kinetis_phrase.s) */
	.equ ST_PCC,           0
	.equ ST_PCR,           4
	.equ ST_PDDR,          8
	.equ ST_PSOR,          12
	.equ ST_PTOR,          16
	.equ ST_MASK,          20
	.equ ST_COUNT_RELOAD,  24
	.equ ST_COUNT_REMAIN,  28

	/* One-time /ST init -- identical to kinetis_phrase.s. Skipped
	 * entirely if r8 == 0. Uses r5-r7 as scratch. */
init_st:
	mov	r5, r8
	cmp	r5, #0
	beq	init_done

	ldr	r6, [r5, #ST_PCC]
	ldr	r7, =0x40000000
	str	r7, [r6]

	ldr	r6, [r5, #ST_PCR]
	ldr	r7, =0x00000100
	str	r7, [r6]

	ldr	r6, [r5, #ST_PDDR]
	ldr	r7, [r6]
	ldr	r5, [r5, #ST_MASK]
	orr	r7, r5
	mov	r5, r8
	ldr	r6, [r5, #ST_PDDR]
	str	r7, [r6]

	ldr	r6, [r5, #ST_PSOR]
	ldr	r7, [r5, #ST_MASK]
	str	r7, [r6]

	ldr	r7, [r5, #ST_COUNT_RELOAD]
	str	r7, [r5, #ST_COUNT_REMAIN]

init_done:

wait_fifo:
	ldr	r6, [r2, #0]		/* read wp */
	cmp	r6, #0
	beq	exit

	ldr	r5, [r2, #4]		/* read rp */
	cmp	r5, r6
	beq	wait_fifo

	/* Inline /ST strobe (same as kinetis_phrase.s) */
	mov	r6, r8
	cmp	r6, #0
	beq	skip_strobe_wf
	ldr	r7, [r6, #ST_COUNT_REMAIN]
	sub	r7, #1
	str	r7, [r6, #ST_COUNT_REMAIN]
	bne	skip_strobe_wf
	ldr	r7, [r6, #ST_COUNT_RELOAD]
	str	r7, [r6, #ST_COUNT_REMAIN]
	ldr	r7, [r6, #ST_PTOR]
	ldr	r6, [r6, #ST_MASK]
	str	r6, [r7]
skip_strobe_wf:

	/* Set up FCCOB for PROGRAM_CHECK:
	 *   FCCOB3..0 := address | (cmd << 24)   (str+strb pair like in
	 *                                         the PHRASE loader)
	 *   FCCOB7..4 := margin << 24  (margin -> FCCOB4, rest = 0)
	 *   FCCOBB..8 := expected longword, byte-reversed so FCCOB8
	 *                 (at high address 0x4002000F) gets the LSB. */
	str	r0, [r4, #FTFx_FCCOB3]	/* writes 4 bytes FCCOB3..0 = addr */
	movs	r7, #CMD_PROGCHECK
	strb	r7, [r4, #FTFx_FCCOB0]	/* overwrite FCCOB0 with cmd byte */

	/* Margin byte at FCCOB4 (which sits at the MSB end of the
	 * 32-bit word at 0x40020008 in our little-endian str). */
	mov	r7, r9
	lsl	r6, r7, #24		/* r6 = margin << 24 */
	str	r6, [r4, #FTFx_FCCOB7]	/* FCCOB7..4 = 0,0,0,margin */

	/* Expected longword from ring buffer, byte-reversed for FCCOB8..11.
	 * (FCCOB8 lives at the HIGH end of the 32-bit slot at 0x4002000C
	 * and holds the expected LSB, so we reverse before storing.) */
	ldr	r6, [r5]
	rev	r6, r6
	str	r6, [r4, #FTFx_FCCOBB]	/* FCCOBB..8 = expected (byte-reversed) */

	/* Launch */
	movs	r7, #FSTAT_CCIF
	strb	r7, [r4, #FTFx_FSTAT]

	/* Advance rp by 4 (one longword) with wrap */
	add	r5, #4
	cmp	r5, r3
	bcc	no_wrap
	mov	r5, r2
	add	r5, #8			/* rp = first byte of data area */
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

	/* CCIF still busy -> strobe and keep waiting */
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
	/* CCIF set. Check FSTAT: MGSTAT0 (bit 0) = margin/program error;
	 * upper error bits (ACCERR/FPVIOL/RDCOLERR) = command setup error.
	 * Either way, fail with the current address still in r0 so the
	 * host can report the failing location. */
	movs	r7, #FSTAT_ERR_MASK
	tst	r6, r7
	bne	error
	movs	r7, #1			/* MGSTAT0 */
	tst	r6, r7
	bne	error

	/* Longword passed margin verify. Move on. */
	add	r0, #4
	sub	r1, #1
	bne	wait_fifo
	b	exit

error:
	movs	r5, #0
	str	r5, [r2, #4]		/* rp = 0 -> signal failure */

exit:
	bkpt	#0

	.ltorg
