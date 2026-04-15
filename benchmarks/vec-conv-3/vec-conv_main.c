// See LICENSE for license details.

//**************************************************************************
// 3x3 2D Convolution Benchmark
//--------------------------------------------------------------------------
//
// This benchmark tests a vectorized 2D 3x3 convolution implementation.

#include <string.h>
#include "util.h"
#include "driver/rocket-chip/l_trace_encoder/l_trace_encoder.h"

//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset1.h"

//--------------------------------------------------------------------------
// Main

void *vec_conv (size_t, size_t, size_t, size_t, const float*, const float*, float*);

int main( int argc, char* argv[] )
{
  LTraceEncoderType *encoder = l_trace_encoder_get(get_hart_id());
  // l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_PREDICT);
  l_trace_encoder_configure_branch_mode(encoder, BRANCH_MODE_TARGET);
  float results_data[O_SIZE] = {0};
  l_trace_encoder_start(encoder);

#if PREALLOCATE
  // If needed we preallocate everything in the caches
  vec_conv(OH, OW, IW, OW, input_k, input_image, results_data);
  memset(results_data, 0, sizeof(results_data));
#endif

  // Do the convolution
  setStats(1);
  vec_conv(OH, OW, IW, OW, input_k, input_image, results_data);
  setStats(0);

  
  l_trace_encoder_stop(encoder);
  // Check the results
  return verifyFloat( O_SIZE, results_data, verify_data );
}
