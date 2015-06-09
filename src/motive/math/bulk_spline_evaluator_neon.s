@ Copyright 2015 Google Inc. All rights reserved.
@
@ Licensed under the Apache License, Version 2.0 (the "License");
@ you may not use this file except in compliance with the License.
@ You may obtain a copy of the License at
@
@     http://www.apache.org/licenses/LICENSE-2.0
@
@ Unless required by applicable law or agreed to in writing, software
@ distributed under the License is distributed on an "AS IS" BASIS,
@ WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@ See the License for the specific language governing permissions and
@ limitations under the License.


  .text
  .syntax   unified

  @ When used with the 'vtbl' instruction, grabs the first byte of every
  @ word, and places it in the first word. Fills the second word with 0s.
  @ For example, (0xFFFFFFFF, 0x00000000, 0x00000000, 0xFFFFFFFF)
  @              ==> (0xFF0000FF, 0x00000000)
kFirstByteTableIndices:
  .byte     0
  .byte     4
  .byte     8
  .byte     12
  .byte     0xFF
  .byte     0xFF
  .byte     0xFF
  .byte     0xFF

  .balign   4
  .global   UpdateCubicXsAndGetMask_Neon
  .thumb_func
UpdateCubicXsAndGetMask_Neon:
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@ void UpdateCubicXsAndGetMask_Neon(
@    const float& delta_x, const float* x_ends, int num_xs, float* xs,
@    uint8_t* masks)
@
@    Parameters
@  r0: *delta_x
@  r1: *x_ends
@  r2: *playback_rates
@  r3: num_xs
@  [sp]: *xs ==> r4
@  [sp + 4]: *masks ==> r5
@
@  q15: kFirstByteTableIndices
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  @ r5 <-- masks
  ldr       r4, [sp]
  ldr       r5, [sp, #4]

  @ q12 <-- delta_x splatted
  vld1.f32  {d24[], d25[]}, [r0]

  @ q15 <-- kFirstByteTableIndices
  adr       r12, kFirstByteTableIndices
  vld1.8    {d30}, [r12]

.L_UpdateCubicXs_Loop:
  @ num_xs -= 4; sets the 'gt' flag used in 'bgt' below
  subs      r3, r3, #4

  @ q8 <-- xs[i]
  @ q9 <-- x_ends[i], x_ends pointer += 4
  @ q11 <-- playback_rate[i], playback_rate pointer += 4
  vld1.f32  {d16, d17}, [r4:128]
  vld1.f32  {d18, d19}, [r1:128]!
  vld1.f32  {d22, d23}, [r2:128]!

  @ q8 <-- xs[i] + x_delta * playback_rate[i]
  vmla.f32  q8, q12, q11

  @ create comparison mask: 0xFFFFFFFF or 0x00000000.
  @ q10 <-- q8 > q9 = xs[i] > x_ends[i] (mask)
  vcgt.f32  q10, q8, q9

  @ xs[i] <-- updated value: xs[i] + delta_x,  xs pointer += 4
  vst1.f32  {d16, d17}, [r4:128]!

  @ pack mask into 4-byte word.
  @ q10[0][1][2][3] <-- q10[0][4][8][12] (byte indices into q10)
  vtbl.8    d20, {d20, d21}, d30

  @ mask[i] <-- q10[0][1][2][3] (write one word), mask pointer += 4
  vst1.32   {d20[0]}, [r5:32]!

  @ continue loop if iterations still remain.
  bgt       .L_UpdateCubicXs_Loop

  @ Return to the address specified in the link register.
  bx        lr


  .balign   4
  .global   EvaluateCubics_Neon
  .thumb_func
EvaluateCubics_Neon:
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@ void EvaluateCubics_Neon(
@    const CubicCurve* cubics, const float* xs, const YRange* y_ranges,
@    int num_cubics, float* ys)
@
@    Parameters
@  r0: *cubics --> also r6 (offset by 32 bytes)
@  r1: *xs
@  r2: *y_ranges --> also r7 (offset by 24 bytes)
@  r3: num_cubics
@  [sp]: *ys (out parameter) --> r4
@
@  r10: 48
@  r12: 16
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  @ save state to stack
  push      {r4-r7, lr}       @ 5 registers x 4 bytes = 20 bytes
  vpush     {q6, q7}         @ 2 registers x 16 bytes = 32 bytes

  @ r4 <-- ys
  ldr       r4, [sp, #52]

  @ r6 <-- cubics + 32 bytes (for efficient loading)
  @ r7 <-- y_ranges + 24 bytes (for efficient loading)
  add       r6, r0, #32
  add       r7, r2, #24

  @ constants
  mov       r10, #48
  mov       r12, #16

  @ q6 <-- 0
  vmov.f32  q6, #0.0

.L_EvaluateCubic_Loop:
  @ num_cubics -= 4
  @ sets the 'gt' flag used in 'bgt' below.
  subs      r3, r3, #4

  @ q8, q9, q10, q11 <-- cubics[i].coeff[0], coeff[1], coeff[2], coefff[3]
  @ deinterlaced the cubic coefficients so each one gets a register.
  vld4.f32  {d16, d18, d20, d22}, [r0:256]
  vld4.f32  {d17, d19, d21, d23}, [r6:256]

  @ q12 <-- xs[i]
  vld1.f32  {d24, d25}, [r1]!

  @ q7 <-- q8 = cubics[i].coeff[0]
  @ save original constant coefficient for later
  vmov      q7, q8

  @ q8 <-- y = c3*x^3 + c2*x^2 + c1*x + c0
  @          = ((c3*x + c2)*x + c1)*x + c0
  vmla.f32  q10, q11, q12   @ q10 = c3*x + c2
  vmla.f32  q9, q10, q12    @ q9 = (c3*x + c2)*x + c1
  vmla.f32  q8, q9, q12     @ q13 = ((c3*x + c2)*x + c1)*x + c0

  @ q9, q10, q11 <-- valid_ys.min, valid_ys.max, modular_arithmetic
  vld3.f32  {d18, d20, d22}, [r2:64], r10
  vld3.f32  {d19, d21, d23}, [r7:64], r10

  @ q12 <-- y <= valid_ys.min (mask)
  @ q13 <-- y > valid_ys.max (mask)
  vcle.f32  q12, q8, q9
  vcgt.f32  q13, q8, q10

  @ q14 <-- valid_ys.length = valid_ys.max - valid_ys.min
  @ q15 <-- -valid_ys.length = valid_ys.min - valid_ys.max
  vsub.f32  q14, q10, q9
  vsub.f32  q15, q9, q10

  @ q12 <-- y_adjustment: one of 0, length, -length
  vbsl      q13, q15, q6    @ y > valid_ys.max ? -length : 0
  vbsl      q12, q14, q13   @ y <= valid_ys.min ? length : (above)
  vbsl      q11, q12, q6    @ modular_arithmetic ? (above) : 0

  @ q8 <-- y + y_adjustment
  @ q7 <-- cubics[i].coeff[0] + y_adjustment
  vadd.f32  q8, q8, q11
  vadd.f32  q7, q7, q11

  @ q8 <-- Clamp(q8, valid_ys.min, valid_ys.max)
  vmax.f32  q8, q8, q9
  vmin.f32  q8, q8, q10

  @ ys[i] <-- q8 = final y, adjusted and clamped
  vst1.f32  {d16, d17}, [r4:128]!

  @ cubics[i].coeff[0] <-- q7 (updated cubic constants).
  vst1.f32  {d14[0]}, [r0:32], r12    @ cubics[0].coeff[0] <-- q7[0], r0 += 16
  vst1.f32  {d15[0]}, [r6:32], r12    @ cubics[2].coeff[0] <-- q7[2], r6 += 16
  vst1.f32  {d14[1]}, [r0:32], r10    @ cubics[1].coeff[0] <-- q7[1], r0 += 48
  vst1.f32  {d15[1]}, [r6:32], r10    @ cubics[3].coeff[0] <-- q7[3], r6 += 48

  @ continue loop if iterations still remain.
  bgt       .L_EvaluateCubic_Loop

  @ Return by popping the link register into the program counter.
  vpop      {q6, q7}
  pop       {r4-r7, pc}
