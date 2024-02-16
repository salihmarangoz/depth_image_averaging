/*
 * This file is part of the depth_image_averaging
 * (https://github.com/salihmarangoz/depth_image_averaging).
 * Copyright (c) 2024 Salih Marangoz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// Parameters
#define MAX_ELEMENTS ({max_elements})
#define MIN_ELEMENTS ({min_elements})
#define TRUE_MEDIAN (true)
#define MAD_UPPER_LIMIT_A (0.008)
#define MAD_UPPER_LIMIT_B (0.0015)
#define MAD_SCALE (1.5)

float nth_element(float *arr, int arr_size, int k) {
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

    if (pivotIndex == k) {
      return arr[pivotIndex];
    } else if (pivotIndex > k) {
      right = pivotIndex - 1;
    } else {
      left = pivotIndex + 1;
    }
  }
  return NAN;
}

__kernel void compute_average(__global const float *input,
                              __global float *output, int size_) {
  size_t gid = get_global_id(0);
  int filtered_size = 0;
  int inf_count = 0;
  float tmp_buffer[MAX_ELEMENTS];

  for (int i = 0; i < size_; i++) {
    float val = input[i + gid * MAX_ELEMENTS];
    if (isfinite(val)) {
      tmp_buffer[filtered_size] = val;
      filtered_size++;
    } else if (isinf(val)) {
      inf_count++;
    }
  }
  int nan_count = size_ - filtered_size - inf_count;

  if (filtered_size < MIN_ELEMENTS) {
    if (nan_count > inf_count) {
      output[gid] = NAN;
    } else {
      output[gid] = INFINITY;
    }
    return;
  }

  // compute median
  float median_value;
  if (TRUE_MEDIAN && filtered_size % 2 == 0) {
    float val_a = nth_element(tmp_buffer, filtered_size, filtered_size / 2);
    float val_b = nth_element(tmp_buffer, filtered_size, filtered_size / 2 - 1);
    median_value = (val_a + val_b) / 2.0;
  } else {
    median_value = nth_element(tmp_buffer, filtered_size, filtered_size / 2);
  }

  // compute mad
  float mad = 0;
  for (int j = 0; j < filtered_size; j++) {
    mad += fabs(tmp_buffer[j] - median_value);
  }
  mad /= filtered_size;

  // MAD scale check
  if (mad > (MAD_UPPER_LIMIT_A * median_value + MAD_UPPER_LIMIT_B)) {
    output[gid] = NAN;
    return;
  }

  // filter outliers and compute mean
  float mad_mean = 0;
  int mad_size = 0;
  for (int j = 0; j < filtered_size; j++) {
    float val = tmp_buffer[j];
    if (isfinite(val) && fabs(val - median_value) < MAD_SCALE * mad) {
      mad_mean += val;
      mad_size++;
    }
  }
  mad_mean /= mad_size;

  // Min-element check
  if (mad_size < MIN_ELEMENTS) {
    output[gid] = NAN;
    return;
  }

  output[gid] = mad_mean;  // TODO: maybe a weighted avg between the mad_mean
                           // and the median value? needs more investigation.
}
