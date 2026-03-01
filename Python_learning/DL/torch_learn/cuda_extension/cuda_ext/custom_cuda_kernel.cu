
#include <torch/types.h>
#include <cuda.h>
#include <cuda_runtime.h>

// CUDA 内核实现：每个元素加上指定的值
__global__ void add_kernel(const float* input, float* output, float value, int size) {
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index < size) {
        output[index] = input[index] + value;
    }
}

// CUDA C++ 函数定义
void custom_cuda_add(const torch::Tensor& input, torch::Tensor& output, float value) {
    auto size = input.numel();
    const int threads = 256;
    const int blocks = (size + threads - 1) / threads;

    add_kernel<<<blocks, threads>>>(
        input.data_ptr<float>(),
        output.data_ptr<float>(),
        value,
        size
    );

    // 确保 CUDA 任务正确完成
    cudaDeviceSynchronize();
}
