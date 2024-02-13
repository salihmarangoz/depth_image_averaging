
// Parameters
#define MAX_ELEMENTS ({max_elements})
#define MIN_ELEMENTS ({min_elements})
#define TRUE_MEDIAN (true)

float nth_element(float *arr, int arr_size, int k)
{
    int left = 0;
    int right = arr_size - 1;
    while (left <= right) {
 
        // partition
        int pivot = arr[right];
        int i = (left - 1);
        for (int j = left; j <= right - 1; j++) {
            if (arr[j] <= pivot) {
                i++;
                // swap
                float tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
        // swap
        float tmp = arr[i + 1];
        arr[i + 1] = arr[right];
        arr[right] = tmp;
        int pivotIndex = i + 1;

        if (pivotIndex == k)
        {
            return arr[pivotIndex];
        }
        else if (pivotIndex > k)
        {
            right = pivotIndex - 1;
        }
        else
        {
            left = pivotIndex + 1;
        }
    }
    return NAN;
}

__kernel void compute_average(__global const float *input, __global float *output, int size_)
{
  size_t gid = get_global_id(0);
  int filtered_size = 0;
  int inf_count = 0;
  float tmp_buffer[MAX_ELEMENTS];

  for (int i = 0; i < size_; i++)
  {
    float val = input[i + gid * MAX_ELEMENTS];
    if (isfinite(val))
    {
      tmp_buffer[filtered_size] = val;
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

  // compute median
  if (TRUE_MEDIAN && filtered_size % 2 == 0)
  {
    float val_a = nth_element(tmp_buffer, filtered_size, filtered_size / 2);
    float val_b = nth_element(tmp_buffer, filtered_size, filtered_size / 2 - 1);
    output[gid] = (val_a + val_b) / 2.0;
  }
  else
  {
    output[gid] = nth_element(tmp_buffer, filtered_size, filtered_size / 2);
  }
}
