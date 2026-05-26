/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * S32K1xx / KE15Z PROGRAM_PHRASE async flash loader.
 *
 * Sister of contrib/loaders/flash/kinetis/kinetis_flash.s (which targets
 * Kinetis K-family PROGRAM_LONGWORD, 4-byte writes). This one targets
 * the FTFC PROGRAM_PHRASE command (0x07) used by S32K1xx and KE15Z:
 * 8 bytes per FCCOB programming cycle.
 *
 * Params at entry (set up by kinetis_write_block_phrase() in kinetis.c
 * via reg_params + target_run_flash_async_algorithm):
 *   r0 = flash destination address (incremented by 8 each phrase)
 *   r1 = phrase count (decremented each phrase, exits at 0)
 *   r2 = ring-buffer descriptor base (wp at [r2,0], rp at [r2,4],
 *        data area starts at [r2,8] and ends just before r3)
 *   r3 = working-area end (ring-buffer wrap point)
 *   r4 = FTFx peripheral base (= 0x40020000)
 *
 * Ring-buffer protocol matches target_run_flash_async_algorithm():
 *  - Host pushes 8 bytes per iteration into the ring at wp, advances wp.
 *  - Algorithm spins until rp != wp, programs the phrase, advances rp.
 *  - Host treats wp == 0 as a stop request (we honour it here too).
 *  - On error we set rp = 0 to signal the host.
 *
 * Bench-measured against the LONGWORD loader's old comment:
 *   old longword sync algo: 6.680 KiB/s @ adapter_khz 2000
 *   longword async algo:    19.808 KiB/s @ adapter_khz 2000
 *   phrase async (this):    target ~40 KiB/s @ adapter_khz 2000
 *   (8 bytes per FTFC cycle vs 4 -> roughly 2x throughput)
 */

	.text
	.cpu cortex-m0plus
	.code 16
	.thumb_func

	.align	2

	/*
	 * Register usage:
	 * r0 = current flash address (input, advances)
	 * r1 = phrases remaining (input, decrements)
	 * r2 = ring-buffer descriptor base (wp/rp pointers + data)
	 * r3 = working-area end
	 * r4 = FTFx peripheral base
	 * r5 = rp (read pointer into ring buffer)
	 * r6 = scratch (wp value, fstat)
	 * r7 = scratch (command byte, data word, masks)
	 */

	/* FTFC register offsets from FTFx base (0x40020000):
	 *   FSTAT  @ 0x00  (1 byte: CCIF + error bits)
	 *   FCCOB3 @ 0x04  (low byte of address)
	 *   FCCOB2 @ 0x05
	 *   FCCOB1 @ 0x06
	 *   FCCOB0 @ 0x07  (command byte -- MUST be written last in byte-by-
	 *                  byte writes; word stores to FCCOB3 then strb to
	 *                  FCCOB0 is the canonical pattern used below)
	 *   FCCOB7 @ 0x08  (byte 0 of first 32-bit data word)
	 *   FCCOB6 @ 0x09
	 *   FCCOB5 @ 0x0A
	 *   FCCOB4 @ 0x0B  (byte 3 of first 32-bit data word)
	 *   FCCOBB @ 0x0C  (byte 0 of second 32-bit data word)
	 *   FCCOBA @ 0x0D
	 *   FCCOB9 @ 0x0E
	 *   FCCOB8 @ 0x0F  (byte 3 of second 32-bit data word)
	 *
	 * Per S32K144 RM §33.4.12.6, PROGRAM_PHRASE wants:
	 *   FCCOB7..FCCOB4 = first 4-byte word, byte 0 (low) at FCCOB7
	 *   FCCOBB..FCCOB8 = second 4-byte word, byte 0 (low) at FCCOBB
	 * which matches little-endian str-to-base layout when we load words
	 * sequentially from the ring buffer.
	 */

	.equ FTFx_FSTAT,  0
	.equ FTFx_FCCOB3, 4
	.equ FTFx_FCCOB0, 7
	.equ FTFx_FCCOB7, 8
	.equ FTFx_FCCOBB, 12

	.equ CMD_PHRASEPROG, 7
	.equ FSTAT_CCIF,     0x80
	.equ FSTAT_ERR_MASK, 0x70   /* ACCERR | FPVIOL | RDCOLERR */

wait_fifo:
	ldr 	r6, [r2, #0]		/* read wp */
	cmp 	r6, #0			/* stop if host set wp == 0 */
	beq 	exit

	ldr 	r5, [r2, #4]		/* read rp */
	cmp 	r5, r6			/* spin until there's a phrase to consume */
	beq 	wait_fifo

	/* Set up FCCOB for PROGRAM_PHRASE
	 *   FCCOB3..0 := address | (cmd << 24) -- do as str+strb pair
	 *   FCCOB7..4 := *rp        (first 4 bytes of phrase)
	 *   FCCOBB..8 := *(rp + 4)  (second 4 bytes of phrase)
	 */
	str	r0, [r4, #FTFx_FCCOB3]	/* writes 4 bytes: FCCOB3..0 = address */
	mov	r7, #CMD_PHRASEPROG
	strb	r7, [r4, #FTFx_FCCOB0]	/* overwrite FCCOB0 with command byte */

	ldr	r7, [r5]		/* first 4 bytes of phrase */
	str	r7, [r4, #FTFx_FCCOB7]

	ldr	r7, [r5, #4]		/* second 4 bytes of phrase */
	str	r7, [r4, #FTFx_FCCOBB]

	/* Launch: write 0x80 (clear CCIF) to FSTAT */
	mov	r7, #FSTAT_CCIF
	strb	r7, [r4, #FTFx_FSTAT]

	/* Advance rp by 8 (one phrase) with ring-buffer wrap */
	add	r5, #8
	cmp     r5, r3			/* wrap if rp reached end of WA */
	bcc     no_wrap
	mov     r5, r2
	add   	r5, #8			/* rp = first byte of data area */

no_wrap:
	str     r5, [r2, #4]		/* store advanced rp */

wait_ccif:
	ldr     r6, [r2, #0]		/* re-check wp (host may have requested stop) */
	cmp     r6, #0
	beq     exit

	ldrb	r6, [r4, #FTFx_FSTAT]	/* poll FSTAT */
	tst	r6, r7			/* r7 still has FSTAT_CCIF (0x80) */
	beq	wait_ccif

	/* CCIF set: check error bits */
	mov	r7, #FSTAT_ERR_MASK
	tst	r6, r7
	bne	error

	/* Phrase succeeded. Move on. */
	add	r0, #8			/* flash address += 8 (only after success!) */

	sub	r1, #1			/* phrase_count-- */
	cmp	r1, #0
	bne	wait_fifo
	b	exit

error:
	mov	r5, #0
	str     r5, [r2, #4]		/* rp = 0 -> signal error to host */

exit:
	bkpt    #0
