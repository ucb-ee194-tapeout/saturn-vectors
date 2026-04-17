// See LICENSE for license details.

//**************************************************************************
// Mixed_width mask benchmark
//--------------------------------------------------------------------------
//
// This benchmark tests a vectorized mixed_width_mask and compute implementation.
// The input data (and reference data) should be generated using
// the mixed_with_mask_gendata.pl perl script and dumped to a file named
// dataset1.h.

#include <string.h>
#include "util.h"
#include "driver/rocket-chip/l_trace_encoder/l_trace_encoder.h"

//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset1.h"

//--------------------------------------------------------------------------
// Main

void vec_mixed_width_mask(size_t n, int8_t x[], int y[], int z[]);


int main( int argc, char* argv[] )
{
  LTraceEncoderType *encoder = l_trace_encoder_get(get_hart_id());
  // l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_PREDICT);
  l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_TARGET);
  l_trace_encoder_start(encoder);
  int results_data[DATA_SIZE];

#if PREALLOCATE
  // If needed we preallocate everything in the caches
  vec_mixed_width_mask(DATA_SIZE, input1_data, results_data, input2_data);
#endif

  // Do the compute
  setStats(1);
  vec_mixed_width_mask(DATA_SIZE, input1_data, results_data, input2_data);
  setStats(0);

  l_trace_encoder_stop(encoder);
  // Check the results
  return verify( DATA_SIZE, results_data, verify_data );
}

