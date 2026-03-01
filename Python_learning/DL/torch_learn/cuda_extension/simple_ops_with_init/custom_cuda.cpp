
#include <torch/extension.h>
#include <vector>

// CUDA 函数声明
void custom_cuda_add(const torch::Tensor& input, torch::Tensor& output, float value);

// PyTorch 前端的绑定函数
torch::Tensor custom_add(torch::Tensor input, float value) {
    auto output = torch::empty_like(input);  // 创建与输入张量相同维度的输出张量
    custom_cuda_add(input, output, value);   // 调用 CUDA 内核函数
    return output;                           // 返回结果张量
}

// Python API 定义
PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
    m.def("custom_add", &custom_add);
}
