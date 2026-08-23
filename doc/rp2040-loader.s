// RP2040 async flash loader. r0 = descriptor base.
// desc: +0 wp (host), +4 rp (target), +8 stop (host), +12 status, +16 fn_erase,
//       +20 fn_program, +24 data_base, +28 reserved, +32 hdr[4]{addr,len,flags,pad}
        .syntax unified
        .cpu cortex-m0plus
        .thumb
        .thumb_func
entry:
        mov     r7, r0
loop:
        ldr     r0, [r7, #0]        // wp
        ldr     r1, [r7, #4]        // rp
        cmp     r0, r1
        beq     check_stop
        movs    r2, #3
        ands    r2, r1              // idx = rp & 3
        lsls    r3, r2, #4
        adds    r3, r3, r7
        adds    r3, #32             // &hdr[idx]
        mov     r6, r3
        ldr     r0, [r6, #8]        // flags
        lsls    r0, r0, #31
        beq     program
        ldr     r0, [r6, #0]        // erase(addr & ~0xfff, 4096, 1<<16, 0xD8)
        lsrs    r0, r0, #12
        lsls    r0, r0, #12
        movs    r1, #1
        lsls    r1, r1, #12
        movs    r2, #1
        lsls    r2, r2, #16
        movs    r3, #0xD8
        ldr     r4, [r7, #16]
        blx     r4
program:
        ldr     r0, [r6, #0]        // addr
        ldr     r1, [r7, #24]       // data_base
        ldr     r2, [r7, #4]        // rp
        movs    r3, #3
        ands    r2, r3
        lsls    r2, r2, #12         // idx * 4096
        adds    r1, r1, r2
        ldr     r2, [r6, #4]        // len
        ldr     r4, [r7, #20]
        blx     r4                  // program(addr, data, len)
        ldr     r1, [r7, #4]
        adds    r1, #1
        str     r1, [r7, #4]        // rp++
        b       loop
check_stop:
        ldr     r0, [r7, #8]
        cmp     r0, #0
        beq     loop
        bkpt    #0
        b       .
