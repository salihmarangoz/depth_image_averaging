
#define MAX_ELEMENTS ({max_elements})
#define MIN_ELEMENTS ({min_elements})

__kernel void compute_average(__global const float *input, __global float *output, int size_)
{
  size_t gid = get_global_id(0);
  int filtered_size = 0;
  float filtered_sum = 0;
  int inf_count = 0;
  for (int i = 0; i < size_; i++)
  {
    float val = input[i + gid * MAX_ELEMENTS];
    if (isfinite(val))
    {
      filtered_sum += val;
      filtered_size++;
    }
    else if (isinf(val))
    {
      inf_count++;
    }
  }
  int nan_count = size_ - filtered_size - inf_count;

  if (filtered_size < MIN_ELEMENTS)
  {
    if (nan_count > inf_count)
    {
      output[gid] = NAN;
    }
    else
    {
      output[gid] = INFINITY;
    }
    return;
  }

  output[gid] = filtered_sum / filtered_size;
}
