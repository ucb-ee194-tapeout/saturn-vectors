/*************************************************************************
* Axpy Kernel
* Author: Jesus Labarta
* Barcelona Supercomputing Center
*************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <riscv_vector.h>
#include "util.h"
#include "driver/rocket-chip/l_trace_encoder/l_trace_encoder.h"

void axpy_intrinsics(double a, double *dx, double *dy, size_t n) {
  for (size_t i = 0; i < n;) {
    long gvl = __riscv_vsetvl_e64m8(n - i);
    vfloat64m8_t v_dx = __riscv_vle64_v_f64m8(&dx[i], gvl);
    vfloat64m8_t v_dy = __riscv_vle64_v_f64m8(&dy[i], gvl);
    vfloat64m8_t v_res = __riscv_vfmacc_vf_f64m8(v_dy, a, v_dx, gvl);
    __riscv_vse64_v_f64m8(&dy[i], v_res, gvl);
    i += gvl;
  }
}


#define N (30 * 1024)
double dx[N];
double dy[N];

int main(int argc, char *argv[])
{
  LTraceEncoderType *encoder = l_trace_encoder_get(get_hart_id());
  // l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_PREDICT);
  l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_TARGET);
  double a=1.0;
  l_trace_encoder_start(encoder);

  // warmup
  axpy_intrinsics(a, dx, dy, N);

  // Start instruction and cycles count of the region of interest
  unsigned long cycles1, cycles2, instr2, instr1;
  instr1 = read_csr(minstret);
  cycles1 = read_csr(mcycle);

  axpy_intrinsics(a, dx, dy, N);

  asm volatile("fence");
  // End instruction and cycles count of the region of interest
  instr2 = read_csr(minstret);
  cycles2 = read_csr(mcycle);
  l_trace_encoder_stop(encoder);
  // Instruction and cycles count of the region of interest
  printf("-CSR   NUMBER OF EXEC CYCLES :%lu\n", cycles2 - cycles1);
  printf("-CSR   NUMBER OF INSTRUCTIONS EXECUTED :%lu\n", instr2 - instr1);
  return 0;
}
