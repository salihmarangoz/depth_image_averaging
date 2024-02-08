
//#define BATCH_SIZE {batch_size}

__kernel void compute_mean(__global const float *input, __global float *output) {
    float sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += input[i + get_global_id(0) * 32];
    }
    output[get_global_id(0)] = sum / 32;
}