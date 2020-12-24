// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include <cuda.h>
#include "grayscale.cuh"
#include <iostream>

namespace chrono {
namespace sensor {

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void mean_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    if (out_index < w * h) {
        // reset buffer to zeros
        bufOut[2 * out_index] = 0;
        bufOut[2 * out_index + 1] = 0;

        float sum_range = 0.f;
        float sum_intensity = 0.f;
        int n_contributing = 0;
        // gather up all of our values, take mean and push to output buffer
        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                sum_intensity += bufIn[2 * in_index + 1];
                if (bufIn[2 * in_index + 1] > 1e-6) {
                    sum_range += bufIn[2 * in_index];
                    n_contributing++;
                }
            }
        }
        if (n_contributing > 0) {
            bufOut[2 * out_index] = sum_range / (n_contributing);
            bufOut[2 * out_index + 1] = sum_intensity / (d * d);
        }
    }
}

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void strong_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    // float* raw_range = new float[d * d];
    // float* raw_intensity = new float[d * d];
    // int raw_id = 0;

    // extract the values we will use in our return distribution
    if (out_index < w * h) {
        float strongest = 0;
        float intensity_at_strongest = 0;

        // perform kernel operation to find max intensity
        float kernel_radius = .05;  // 10 cm total kernel width

        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                // float range = bufIn[2 * in_index];
                // float intensity = bufIn[2 * in_index + 1];

                float local_range = bufIn[2 * in_index];
                float local_intensity = bufIn[2 * in_index + 1];

                for (int k = 0; k < d; k++) {
                    for (int l = 0; l < d; l++) {
                        int inner_in_index = (d * out_vIndex + k) * d * w + (d * out_hIndex + l);
                        float range = bufIn[2 * inner_in_index];
                        float intensity = bufIn[2 * inner_in_index + 1];

                        if (inner_in_index != in_index && abs(range - local_range) < kernel_radius) {
                            float weight = (kernel_radius - abs(range - local_range)) / kernel_radius;
                            local_intensity += weight * intensity;
                            // norm_val += weight;
                        }
                    }
                }

                local_intensity = local_intensity / (d * d);  // calculating portion of beam here
                if (local_intensity > intensity_at_strongest) {
                    intensity_at_strongest = local_intensity;
                    strongest = local_range;
                }

                // raw_range[raw_id] = bufIn[2 * in_index];
                // raw_intensity[raw_id] = bufIn[2 * in_index + 1];

                // if (raw_id > d * d)
                //     printf("OH NO!\n");
                // raw_id++;
            }
        }

        bufOut[2 * out_index] = strongest;
        bufOut[2 * out_index + 1] = intensity_at_strongest;
    }

    // // essentially performing a linear blur to find range of max intensity
    // for (int i = 0; i < d * d; i++) {
    //     float norm_val = 1;
    //     float local_intensity = raw_intensity[i];
    //     for (int j = 0; j < d * d; j++) {
    //         if (j != i && abs(raw_range[j] - raw_range[i]) < kernel_radius) {
    //             float weight = (kernel_radius - abs(raw_range[j] - raw_range[i])) / kernel_radius;
    //             local_intensity += weight * raw_intensity[j];
    //             norm_val += weight;
    //         }
    //     }
    //     local_intensity = local_intensity / (d * d);  // calculating portion of beam here
    //     if (local_intensity > intensity_at_strongest) {
    //         intensity_at_strongest = local_intensity;
    //         strongest = raw_range[i];
    //     }
    // }
    //
    // // push strongest return
    // bufOut[2 * out_index] = strongest;
    // bufOut[2 * out_index + 1] = intensity_at_strongest;
    // //
    // delete[] raw_range;
    // delete[] raw_intensity;
}

void cuda_lidar_mean_reduce(void* bufIn, void* bufOut, int width, int height, int radius) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;

    // printf("buffer dimensions: %d,%d\n", w, h);

    // in one shot - each kernel does O(r^2):
    mean_reduce_kernel<<<nBlocks, nThreads>>>((float*)bufIn, (float*)bufOut, w, h, radius);
    // in two shots - each kernel does O(r)
}

void cuda_lidar_strong_reduce(void* bufIn, void* bufOut, int width, int height, int radius) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;

    // printf("buffer dimensions: %d,%d\n", w, h);

    // in one shot - each kernel does O(r^2):
    strong_reduce_kernel<<<nBlocks, nThreads>>>((float*)bufIn, (float*)bufOut, w, h, radius);
    // in two shots - each kernel does O(r)
}

}  // namespace sensor
}  // namespace chrono
