// ============================================================
// YOLO11 TensorRT 推理模块 —— 带 CUDA 前/后处理加速
// YOLO11 TensorRT inference module with CUDA pre/post processing
// ============================================================

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

// ----------------------------------------------------------
// TensorRT 日志回调 —— 只输出 WARNING 及以上级别
// TensorRT logger callback — outputs WARNING and above only
// ----------------------------------------------------------
class TRTLogger : public nvinfer1::ILogger {
    void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[YOLO11TRT_OPT] " << msg << std::endl;
        }
    }
};

static TRTLogger g_logger;

// ----------------------------------------------------------
// 工具函数：以二进制方式读取整个文件到内存
// Utility: read entire binary file into memory
// ----------------------------------------------------------
static std::vector<char> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open())
        throw std::runtime_error("Failed to open file: " + path);
    size_t size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    f.read(buffer.data(), static_cast<std::streamsize>(size));
    return buffer;
}

// ----------------------------------------------------------
// 工具函数：封装 CUDA 错误检查，失败时抛异常
// Utility: wrap CUDA error check, throw on failure
// ----------------------------------------------------------
static void cudaCheck(cudaError_t e, const char* msg) {
    if (e != cudaSuccess) {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(e));
    }
}

// ----------------------------------------------------------
// 检测结果结构体（原始坐标 + 置信度 + 类别）
// Detection result struct (raw coordinates + confidence + class)
// ----------------------------------------------------------
struct Detection {
    float x1, y1, x2, y2;  // 检测框左上/右下坐标 (原始图像坐标系)
    float score;            // 置信度
    int class_id;           // 类别 ID
};

// ----------------------------------------------------------
// CUDA 核函数：letterbox 缩放 + BGR→CHW 转换 + 归一化 [0,1]
// 功能：保持宽高比缩放原始图像到目标尺寸，对不足部分填充 0，
//       同时完成 HWC→CHW 维度变换和双线性插值
// ----------------------------------------------------------
// src:  原始图像 (H x W x 3, uint8)
// dst:  输出张量 (3 x dst_h x dst_w, float32)
// ----------------------------------------------------------
__global__ void letterboxBgrToChwKernel(
    const uint8_t* src, int src_h, int src_w,
    float* dst, int dst_h, int dst_w,
    float scale, int resize_h, int resize_w, int pad_y, int pad_x)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = 3 * dst_h * dst_w;
    if (idx >= total) return;

    // 计算当前线程对应的通道、行、列
    int plane = dst_h * dst_w;
    int c = idx / plane;
    int rem = idx - c * plane;
    int y = rem / dst_w;
    int x = rem - y * dst_w;

    // padding 区域直接填 0
    if (x < pad_x || x >= pad_x + resize_w ||
        y < pad_y || y >= pad_y + resize_h) {
        dst[idx] = 0.0f;
        return;
    }

    // 双线性插值映射回原始图像坐标
    float src_x = (x - pad_x) / scale;
    float src_y = (y - pad_y) / scale;
    int x0 = static_cast<int>(src_x);
    int y0 = static_cast<int>(src_y);
    x0 = max(0, min(x0, src_w - 1));
    y0 = max(0, min(y0, src_h - 1));
    int x1 = min(x0 + 1, src_w - 1);
    int y1 = min(y0 + 1, src_h - 1);
    float fx = src_x - x0;
    float fy = src_y - y0;

    // 双线性插值并归一化到 [0, 1]
    float v00 = src[(y0 * src_w + x0) * 3 + c];
    float v10 = src[(y0 * src_w + x1) * 3 + c];
    float v01 = src[(y1 * src_w + x0) * 3 + c];
    float v11 = src[(y1 * src_w + x1) * 3 + c];
    float v = (1.0f - fy) * (1.0f - fx) * v00 +
              (1.0f - fy) * fx * v10 +
              fy * (1.0f - fx) * v01 +
              fy * fx * v11;
    dst[idx] = v / 255.0f;
}

// ----------------------------------------------------------
// CUDA 核函数：解析 YOLO 输出张量 → 检测框列表
// 功能：从模型输出中解析边界框 (cx,cy,w,h)、计算最高分类别、
//       完成归一化坐标 → 原始图像坐标的反向映射，通过原子操作收集结果
// ----------------------------------------------------------
// data:       模型原始输出 (float*)
// transposed: true= [num_boxes, 4+num_classes], false= [4+num_classes, num_boxes]
// 归一化判断：若框坐标值 ≤2.0 视为归一化值，需乘以 (tW, tH) 还原
// ----------------------------------------------------------
__global__ void decodeYoloKernel(
    const float* data, int num_boxes, int num_classes, bool transposed,
    int tH, int tW, float scale, float pad_x, float pad_y,
    int orig_w, int orig_h, float conf_thresh,
    Detection* dets, int* det_count, int max_dets)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_boxes) return;

    int stride = 4 + num_classes;
    float cx, cy, bw, bh;

    // 根据张量布局读取 (cx, cy, w, h)
    if (transposed) {
        const float* row = data + i * stride;
        cx = row[0];
        cy = row[1];
        bw = row[2];
        bh = row[3];
    } else {
        cx = data[i];
        cy = data[1 * num_boxes + i];
        bw = data[2 * num_boxes + i];
        bh = data[3 * num_boxes + i];
    }

    // 找最大置信度的类别（支持自动 sigmoid 校准）
    float max_score = 0.0f;
    int max_id = -1;
    for (int j = 0; j < num_classes; ++j) {
        float s = transposed ? data[i * stride + 4 + j]
                             : data[(4 + j) * num_boxes + i];
        if (s < 0.0f || s > 1.0f) {
            s = 1.0f / (1.0f + expf(-s));
        }
        if (s > max_score) {
            max_score = s;
            max_id = j;
        }
    }
    if (max_score < conf_thresh) return;

    // (cx,cy,w,h) → (x1,y1,x2,y2)
    float x1 = cx - bw * 0.5f;
    float y1 = cy - bh * 0.5f;
    float x2 = cx + bw * 0.5f;
    float y2 = cy + bh * 0.5f;

    float max_box_value = fmaxf(fmaxf(fabsf(cx), fabsf(cy)),
                                fmaxf(fabsf(bw), fabsf(bh)));
    // 若坐标在 [0,1] 或 [0,2] 范围，视为归一化坐标，还原到网络输入分辨率
    if (max_box_value <= 2.0f) {
        x1 *= tW;
        x2 *= tW;
        y1 *= tH;
        y2 *= tH;
    }

    // 反向 letterbox: 去除填充并缩放到原始图像尺寸
    float ox1 = (x1 - pad_x) / scale;
    float oy1 = (y1 - pad_y) / scale;
    float ox2 = (x2 - pad_x) / scale;
    float oy2 = (y2 - pad_y) / scale;

    // 钳位到原始图像边界
    ox1 = fmaxf(0.0f, fminf(ox1, static_cast<float>(orig_w - 1)));
    oy1 = fmaxf(0.0f, fminf(oy1, static_cast<float>(orig_h - 1)));
    ox2 = fmaxf(0.0f, fminf(ox2, static_cast<float>(orig_w - 1)));
    oy2 = fmaxf(0.0f, fminf(oy2, static_cast<float>(orig_h - 1)));

    // 通过原子操作将结果写入输出数组（线程安全）
    int out_idx = atomicAdd(det_count, 1);
    if (out_idx < max_dets) {
        dets[out_idx] = {ox1, oy1, ox2, oy2, max_score, max_id};
    }
}

// ============================================================
// YOLO11TRT 主类 —— TensorRT 推理 + CUDA 前/后处理
// 支持两条路径：
//   1) CPU 路径：preprocess() → infer_raw() → postprocess()
//   2) CUDA 路径：detect_cuda() 端到端 GPU 加速
// ============================================================
class YOLO11TRT {
public:
    // ----------------------------------------------------------
    // 构造函数：加载 engine + 分配 GPU 资源
    // Constructor: load engine + allocate GPU resources
    // ----------------------------------------------------------
    YOLO11TRT(const std::string& engine_path,
              float conf_thresh = 0.5f,
              float iou_thresh = 0.5f,
              int num_classes = 80)
        : conf_thresh_(conf_thresh), iou_thresh_(iou_thresh), num_classes_(num_classes)
    {
        // 1) 创建 TensorRT runtime
        runtime_.reset(nvinfer1::createInferRuntime(g_logger));
        if (!runtime_) throw std::runtime_error("createInferRuntime failed");

        // 2) 从文件反序列化 engine
        auto buf = readFile(engine_path);
        engine_.reset(runtime_->deserializeCudaEngine(buf.data(), buf.size()));
        if (!engine_) throw std::runtime_error("deserializeCudaEngine failed");

        // 3) 创建执行上下文
        context_.reset(engine_->createExecutionContext());
        if (!context_) throw std::runtime_error("createExecutionContext failed");

        // 4) 枚举所有 IO tensor，记录输入/输出名和形状
        int nb = engine_->getNbIOTensors();
        for (int i = 0; i < nb; ++i) {
            auto const* name = engine_->getIOTensorName(i);
            auto mode = engine_->getTensorIOMode(name);
            auto dims = engine_->getTensorShape(name);
            if (mode == nvinfer1::TensorIOMode::kINPUT) {
                input_name_ = name;
                input_dims_ = dims;
            } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
                output_name_ = name;
                output_dims_ = dims;
            }
        }
        if (input_name_.empty() || output_name_.empty())
            throw std::runtime_error("Engine missing input or output tensor");

        // 5) 计算元素总数并推断输出布局（transposed / non-transposed）
        input_size_ = volume(input_dims_);
        output_size_ = volume(output_dims_);
        parseOutputLayout();

        // 6) 分配 GPU 资源
        cudaCheck(cudaMalloc(&input_dev_, input_size_ * sizeof(float)), "cudaMalloc input");
        cudaCheck(cudaMalloc(&output_dev_, output_size_ * sizeof(float)), "cudaMalloc output");
        cudaCheck(cudaMalloc(&decode_dets_dev_, max_dets_ * sizeof(Detection)), "cudaMalloc decode_dets");
        cudaCheck(cudaMalloc(&decode_count_dev_, sizeof(int)), "cudaMalloc decode_count");
        cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");

        // 7) 绑定 IO tensor 地址
        context_->setTensorAddress(input_name_.c_str(), input_dev_);
        context_->setTensorAddress(output_name_.c_str(), output_dev_);

        // 8) 打印加载信息
        std::cout << "[YOLO11TRT_OPT] Engine loaded\n"
                  << "  Input  : " << input_name_ << "  shape=";
        printDims(input_dims_);
        std::cout << "\n  Output : " << output_name_ << "  shape=";
        printDims(output_dims_);
        std::cout << "\n  num_classes=" << num_classes_
                  << "  conf_thresh=" << conf_thresh_
                  << "  iou_thresh=" << iou_thresh_ << std::endl;
    }

    // ----------------------------------------------------------
    // 析构函数：释放所有 GPU 资源
    // Destructor: release all GPU resources
    // ----------------------------------------------------------
    ~YOLO11TRT() {
        if (stream_) cudaStreamDestroy(stream_);
        if (input_dev_) cudaFree(input_dev_);
        if (output_dev_) cudaFree(output_dev_);
        if (image_dev_) cudaFree(image_dev_);
        if (decode_dets_dev_) cudaFree(decode_dets_dev_);
        if (decode_count_dev_) cudaFree(decode_count_dev_);
    }

    // 禁止拷贝
    YOLO11TRT(const YOLO11TRT&) = delete;
    YOLO11TRT& operator=(const YOLO11TRT&) = delete;

    // ----------------------------------------------------------
    // CPU 前处理：letterbox + BGR→CHW + 归一化 [0,1]
    // 返回预处理张量及还原所需的 scale/pad/orig 信息
    // CPU preprocess: letterbox + HWC→CHW + normalize [0,1]
    // ----------------------------------------------------------
    py::dict preprocess(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> image) {
        auto buf = image.request();
        validateImage(buf);
        int H = static_cast<int>(buf.shape[0]);   // 原始高
        int W = static_cast<int>(buf.shape[1]);   // 原始宽
        int tH = static_cast<int>(input_dims_.d[2]);  // 目标高 (网络输入)
        int tW = static_cast<int>(input_dims_.d[3]);  // 目标宽 (网络输入)
        // 保持宽高比的缩放系数
        float scale = std::min(static_cast<float>(tW) / W, static_cast<float>(tH) / H);
        int nW = static_cast<int>(W * scale);     // 缩放后宽
        int nH = static_cast<int>(H * scale);     // 缩放后高
        // padding 偏移量（居中填充）
        float pad_x = (tW - nW) / 2.0f;
        float pad_y = (tH - nH) / 2.0f;

        auto result = py::array_t<float>({3, tH, tW});
        auto rbuf = result.request();
        preprocessToHost(static_cast<uint8_t const*>(buf.ptr), H, W,
                         static_cast<float*>(rbuf.ptr), tH, tW,
                         scale, nH, nW, static_cast<int>(pad_y), static_cast<int>(pad_x));
        return makePreprocInfo(result, scale, pad_x, pad_y, W, H);
    }

    // ----------------------------------------------------------
    // 纯推理（输入已是 float CHW 张量）
    // 用于 CPU 路径：将主机数据拷到 GPU → 执行 → 拷回主机
    // ----------------------------------------------------------
    py::array_t<float> infer_raw(py::array_t<float, py::array::c_style | py::array::forcecast> input_data) {
        auto buf = input_data.request();
        if (static_cast<size_t>(buf.size) != input_size_)
            throw std::runtime_error("Input size mismatch: expected " + std::to_string(input_size_) +
                                     " floats, got " + std::to_string(buf.size));
        cudaCheck(cudaMemcpyAsync(input_dev_, buf.ptr, input_size_ * sizeof(float),
                                  cudaMemcpyHostToDevice, stream_), "cudaMemcpyAsync H2D input");
        enqueue();
        return copyOutputToHost();
    }

    // ----------------------------------------------------------
    // CPU 后处理：解码输出张量 → NMS → 检测框列表
    // 需传入 preprocess() 返回的预处理信息以还原坐标
    // ----------------------------------------------------------
    py::list postprocess(py::array_t<float, py::array::c_style | py::array::forcecast> raw_output,
                         py::dict preproc_info) {
        auto buf = raw_output.request();
        auto* data = static_cast<float const*>(buf.ptr);
        float scale = preproc_info["scale"].cast<float>();
        float pad_x = preproc_info["pad_x"].cast<float>();
        float pad_y = preproc_info["pad_y"].cast<float>();
        int ow = preproc_info["orig_w"].cast<int>();
        int oh = preproc_info["orig_h"].cast<int>();
        return postprocessHost(data, scale, pad_x, pad_y, ow, oh);
    }

    // ----------------------------------------------------------
    // CPU 端到端推理（CPU preprocess → GPU infer → CPU postprocess）
    // 适用于数据量小或需要与纯 Python 前/后处理对齐的场景
    // ----------------------------------------------------------
    py::list infer(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> image) {
        auto pre = preprocess(image);
        auto raw = infer_raw(pre["data"].cast<py::array_t<float>>());
        return postprocess(raw, pre);
    }

    // ----------------------------------------------------------
    // CUDA 端到端推理（全流程 GPU 加速）
    // 步骤：1) H2D 图像  2) GPU letterbox+CHW  3) enqueue
    //       4) GPU decode  5) D2H 检测结果  6) CPU NMS
    // ----------------------------------------------------------
    py::list detect_cuda(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> image) {
        auto buf = image.request();
        validateImage(buf);
        int H = static_cast<int>(buf.shape[0]);
        int W = static_cast<int>(buf.shape[1]);
        int tH = static_cast<int>(input_dims_.d[2]);
        int tW = static_cast<int>(input_dims_.d[3]);
        float scale = std::min(static_cast<float>(tW) / W, static_cast<float>(tH) / H);
        int nW = static_cast<int>(W * scale);
        int nH = static_cast<int>(H * scale);
        float pad_x = (tW - nW) / 2.0f;
        float pad_y = (tH - nH) / 2.0f;

        // Step 1: 上传原始图像到 GPU
        ensureImageBuffer(static_cast<size_t>(H) * W * 3);
        cudaCheck(cudaMemcpyAsync(image_dev_, buf.ptr, image_bytes_, cudaMemcpyHostToDevice, stream_),
                  "cudaMemcpyAsync H2D image");

        // Step 2: GPU letterbox + BGR→CHW + 归一化 [0,1]
        int total = 3 * tH * tW;
        int block = 256;
        int grid = (total + block - 1) / block;
        letterboxBgrToChwKernel<<<grid, block, 0, stream_>>>(
            image_dev_, H, W, static_cast<float*>(input_dev_), tH, tW,
            scale, nH, nW, static_cast<int>(pad_y), static_cast<int>(pad_x));
        cudaCheck(cudaGetLastError(), "letterboxBgrToChwKernel");

        // Step 3: TensorRT 推理
        enqueue();

        // Step 4: GPU 解码（输出张量 → 原始坐标系检测框）
        cudaCheck(cudaMemsetAsync(decode_count_dev_, 0, sizeof(int), stream_), "cudaMemset decode_count");
        int decode_grid = (num_boxes_ + block - 1) / block;
        decodeYoloKernel<<<decode_grid, block, 0, stream_>>>(
            static_cast<float const*>(output_dev_), num_boxes_, num_classes_, transposed_,
            tH, tW, scale, pad_x, pad_y, W, H, conf_thresh_,
            static_cast<Detection*>(decode_dets_dev_), static_cast<int*>(decode_count_dev_), max_dets_);
        cudaCheck(cudaGetLastError(), "decodeYoloKernel");

        // Step 5: 读取检测结果回主机
        int det_count = 0;
        cudaCheck(cudaMemcpyAsync(&det_count, decode_count_dev_, sizeof(int), cudaMemcpyDeviceToHost, stream_),
                  "cudaMemcpyAsync D2H det_count");
        cudaCheck(cudaStreamSynchronize(stream_), "cudaStreamSynchronize det_count");
        det_count = std::max(0, std::min(det_count, max_dets_));

        std::vector<Detection> dets(static_cast<size_t>(det_count));
        if (det_count > 0) {
            cudaCheck(cudaMemcpyAsync(dets.data(), decode_dets_dev_, det_count * sizeof(Detection),
                                      cudaMemcpyDeviceToHost, stream_), "cudaMemcpyAsync D2H dets");
            cudaCheck(cudaStreamSynchronize(stream_), "cudaStreamSynchronize dets");
        }

        // Step 6: CPU NMS 去除冗余框
        auto keep = nms(dets);
        return toPyList(keep);
    }

    // ----------------------------------------------------------
    // infer_cuda 是 detect_cuda 的别名
    // ----------------------------------------------------------
    py::list infer_cuda(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> image) {
        return detect_cuda(image);
    }

    // ----------------------------------------------------------
    // 属性访问器
    // ----------------------------------------------------------
    std::string get_input_name() const { return input_name_; }
    std::string get_output_name() const { return output_name_; }
    py::tuple get_input_shape() const { return py::make_tuple(input_dims_.d[2], input_dims_.d[3]); }
    py::tuple get_output_shape() const {
        py::tuple t(output_dims_.nbDims);
        for (int i = 0; i < output_dims_.nbDims; ++i) t[i] = output_dims_.d[i];
        return t;
    }

private:
// ==================== 私有成员 ====================

    // TensorRT 核心对象
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = nullptr;

    // IO tensor 元信息
    std::string input_name_;
    std::string output_name_;
    nvinfer1::Dims input_dims_{};
    nvinfer1::Dims output_dims_{};
    size_t input_size_ = 0;     // 输入元素总数 (C*H*W)
    size_t output_size_ = 0;    // 输出元素总数

    // GPU 设备内存指针
    void* input_dev_ = nullptr;         // 预处理后的输入张量
    void* output_dev_ = nullptr;        // 原始输出张量
    uint8_t* image_dev_ = nullptr;      // 原始图像 (HWC, uint8)
    size_t image_capacity_ = 0;         // 当前 GPU 图像缓冲容量
    size_t image_bytes_ = 0;            // 当前图像字节数
    void* decode_dets_dev_ = nullptr;   // GPU 解码结果 (Detection[])
    void* decode_count_dev_ = nullptr;  // GPU 检测计数 (int)

    // 推理参数
    float conf_thresh_;
    float iou_thresh_;
    int num_classes_;
    int num_boxes_ = 0;        // 每张图的边界框数
    int max_dets_ = 0;         // 最大检测数上限
    bool transposed_ = false;  // 输出布局是否为 [N, 4+num_classes]

// ==================== 私有工具方法 ====================

    // 计算 Dims 全维度元素数
    static size_t volume(nvinfer1::Dims const& d) {
        size_t v = 1;
        for (int i = 0; i < d.nbDims; ++i) v *= d.d[i];
        return v;
    }

    // 打印 Dims 形状，如 "1x84x8400"
    static void printDims(nvinfer1::Dims const& d) {
        for (int i = 0; i < d.nbDims; ++i)
            std::cout << d.d[i] << (i + 1 < d.nbDims ? "x" : "");
    }

    // ----------------------------------------------------------
    // 推断输出张量布局：transposed ([N, 4+C]) 或 non-transposed ([4+C, N])
    // 支持的 shape 示例：
    //   [1, 84, 8400] → d[-2]==84 → non-transposed, num_boxes=8400
    //   [1, 8400, 84] → d[-1]==84 → transposed,    num_boxes=8400
    //   [8400, 84]    → d[1]==84   → transposed,    num_boxes=8400
    // ----------------------------------------------------------
    void parseOutputLayout() {
        if (output_dims_.nbDims >= 3 &&
            output_dims_.d[output_dims_.nbDims - 2] == 4 + num_classes_) {
            num_boxes_ = static_cast<int>(output_dims_.d[output_dims_.nbDims - 1]);
            transposed_ = false;
        } else if (output_dims_.nbDims >= 3 &&
                   output_dims_.d[output_dims_.nbDims - 1] == 4 + num_classes_) {
            num_boxes_ = static_cast<int>(output_dims_.d[output_dims_.nbDims - 2]);
            transposed_ = true;
        } else if (output_dims_.nbDims == 2 && output_dims_.d[1] == 4 + num_classes_) {
            num_boxes_ = static_cast<int>(output_dims_.d[0]);
            transposed_ = true;
        } else {
            throw std::runtime_error("Cannot infer output layout from tensor shape");
        }
        max_dets_ = std::max(1, num_boxes_);
    }

    // 校验输入图像格式：必须为 3 维 (H, W, 3)
    static void validateImage(py::buffer_info const& buf) {
        if (buf.ndim != 3) throw std::runtime_error("image must be 3D (H, W, C)");
        if (static_cast<int>(buf.shape[2]) != 3) throw std::runtime_error("image must have 3 channels");
    }

    // 确保 GPU 图像缓冲足够大，不足则重新分配
    void ensureImageBuffer(size_t bytes) {
        image_bytes_ = bytes;
        if (bytes <= image_capacity_) return;
        if (image_dev_) cudaFree(image_dev_);
        cudaCheck(cudaMalloc(&image_dev_, bytes), "cudaMalloc image_dev");
        image_capacity_ = bytes;
    }

    // 异步执行 TensorRT 推理
    void enqueue() {
        if (!context_->enqueueV3(stream_)) throw std::runtime_error("enqueueV3 failed");
    }

    // 将 GPU 上的原始输出拷贝到主机 numpy array
    py::array_t<float> copyOutputToHost() {
        std::vector<py::ssize_t> shape;
        for (int d = 0; d < output_dims_.nbDims; ++d)
            shape.push_back(static_cast<py::ssize_t>(output_dims_.d[d]));
        auto result = py::array_t<float>(shape);
        auto rbuf = result.request();
        cudaCheck(cudaMemcpyAsync(rbuf.ptr, output_dev_, output_size_ * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream_), "cudaMemcpyAsync D2H output");
        cudaCheck(cudaStreamSynchronize(stream_), "cudaStreamSynchronize output");
        return result;
    }

    // 将预处理参数打包为 dict，供后处理使用
    static py::dict makePreprocInfo(py::array_t<float>& data, float scale, float pad_x,
                                    float pad_y, int orig_w, int orig_h) {
        py::dict info;
        info["data"] = data;
        info["scale"] = scale;
        info["pad_x"] = pad_x;
        info["pad_y"] = pad_y;
        info["orig_w"] = orig_w;
        info["orig_h"] = orig_h;
        return info;
    }

    // CPU 版 letterbox + HWC→CHW + 归一化 [0,1]（双线性插值）
    static void preprocessToHost(const uint8_t* src, int H, int W, float* dst,
                                 int tH, int tW, float scale, int nH, int nW,
                                 int pad_y, int pad_x) {
        std::fill(dst, dst + 3 * tH * tW, 0.0f);
        for (int c = 0; c < 3; ++c) {
            for (int y = 0; y < nH; ++y) {
                for (int x = 0; x < nW; ++x) {
                    float sx = x / scale;
                    float sy = y / scale;
                    int x0 = static_cast<int>(sx);
                    int y0 = static_cast<int>(sy);
                    int x1 = std::min(x0 + 1, W - 1);
                    int y1 = std::min(y0 + 1, H - 1);
                    float fx = sx - x0;
                    float fy = sy - y0;
                    float v00 = src[y0 * W * 3 + x0 * 3 + c];
                    float v10 = src[y0 * W * 3 + x1 * 3 + c];
                    float v01 = src[y1 * W * 3 + x0 * 3 + c];
                    float v11 = src[y1 * W * 3 + x1 * 3 + c];
                    float v = (1 - fy) * (1 - fx) * v00 + (1 - fy) * fx * v10 +
                              fy * (1 - fx) * v01 + fy * fx * v11;
                    int dy = y + pad_y;
                    int dx = x + pad_x;
                    dst[c * tH * tW + dy * tW + dx] = v / 255.0f;
                }
            }
        }
    }

    // ----------------------------------------------------------
    // CPU 后处理核心：解析原始输出 → NMS → 检测框列表
    // CPU postprocess core: decode raw output → NMS → detection list
    // ----------------------------------------------------------
    py::list postprocessHost(const float* data, float scale, float pad_x, float pad_y,
                             int ow, int oh) const {
        int tW = static_cast<int>(input_dims_.d[3]);
        int tH = static_cast<int>(input_dims_.d[2]);
        int stride = 4 + num_classes_;
        std::vector<Detection> dets;
        dets.reserve(static_cast<size_t>(num_boxes_));
        for (int i = 0; i < num_boxes_; ++i) {
            float cx, cy, bw, bh;
            if (transposed_) {
                cx = data[i * stride + 0];
                cy = data[i * stride + 1];
                bw = data[i * stride + 2];
                bh = data[i * stride + 3];
            } else {
                cx = data[i];
                cy = data[1 * num_boxes_ + i];
                bw = data[2 * num_boxes_ + i];
                bh = data[3 * num_boxes_ + i];
            }
            float max_score = 0.0f;
            int max_id = -1;
            for (int j = 0; j < num_classes_; ++j) {
                float s = transposed_ ? data[i * stride + 4 + j]
                                      : data[(4 + j) * num_boxes_ + i];
                if (s < 0.0f || s > 1.0f) s = 1.0f / (1.0f + std::exp(-s));
                if (s > max_score) { max_score = s; max_id = j; }
            }
            if (max_score < conf_thresh_) continue;
            float x1 = cx - bw / 2.0f;
            float y1 = cy - bh / 2.0f;
            float x2 = cx + bw / 2.0f;
            float y2 = cy + bh / 2.0f;
            float max_box = std::max(std::max(std::fabs(cx), std::fabs(cy)),
                                     std::max(std::fabs(bw), std::fabs(bh)));
            if (max_box <= 2.0f) { x1 *= tW; x2 *= tW; y1 *= tH; y2 *= tH; }
            float ox1 = (x1 - pad_x) / scale;
            float oy1 = (y1 - pad_y) / scale;
            float ox2 = (x2 - pad_x) / scale;
            float oy2 = (y2 - pad_y) / scale;
            ox1 = std::max(0.0f, std::min(ox1, static_cast<float>(ow - 1)));
            oy1 = std::max(0.0f, std::min(oy1, static_cast<float>(oh - 1)));
            ox2 = std::max(0.0f, std::min(ox2, static_cast<float>(ow - 1)));
            oy2 = std::max(0.0f, std::min(oy2, static_cast<float>(oh - 1)));
            dets.push_back({ox1, oy1, ox2, oy2, max_score, max_id});
        }
        auto keep = nms(dets);
        return toPyList(keep);
    }

    // ----------------------------------------------------------
    // 非极大值抑制 (NMS) — 按置信度降序，去除 IoU 过高的冗余框
    // ----------------------------------------------------------
    std::vector<Detection> nms(std::vector<Detection>& dets) const {
        if (dets.empty()) return {};
        // 按置信度从高到低排序
        std::sort(dets.begin(), dets.end(),
                  [](Detection const& a, Detection const& b) { return a.score > b.score; });
        std::vector<bool> suppressed(dets.size(), false);
        std::vector<Detection> keep;
        keep.reserve(dets.size());
        for (size_t i = 0; i < dets.size(); ++i) {
            if (suppressed[i]) continue;
            keep.push_back(dets[i]);
            for (size_t j = i + 1; j < dets.size(); ++j) {
                if (suppressed[j]) continue;
                // 计算交集区域
                float ix1 = std::max(dets[i].x1, dets[j].x1);
                float iy1 = std::max(dets[i].y1, dets[j].y1);
                float ix2 = std::min(dets[i].x2, dets[j].x2);
                float iy2 = std::min(dets[i].y2, dets[j].y2);
                float iw = std::max(0.0f, ix2 - ix1);
                float ih = std::max(0.0f, iy2 - iy1);
                float inter = iw * ih;                               // 交集面积
                float ai = (dets[i].x2 - dets[i].x1) * (dets[i].y2 - dets[i].y1);
                float aj = (dets[j].x2 - dets[j].x1) * (dets[j].y2 - dets[j].y1);
                float iou = inter / (ai + aj - inter + 1e-8f);       // IoU
                if (iou > iou_thresh_) suppressed[j] = true;
            }
        }
        return keep;
    }

    // ----------------------------------------------------------
    // 将 Detection 列表转为 Python list[dict] 返回给调用方
    // ----------------------------------------------------------
    static py::list toPyList(const std::vector<Detection>& dets) {
        py::list result;
        for (auto const& d : dets) {
            py::dict item;
            item["x1"] = d.x1;
            item["y1"] = d.y1;
            item["x2"] = d.x2;
            item["y2"] = d.y2;
            item["score"] = d.score;
            item["class_id"] = d.class_id;
            result.append(item);
        }
        return result;
    }
};

// ============================================================
// Python 模块绑定 (pybind11) — 生成 Yolo11DetTrt 模块
// 暴露的类：YOLO11TRT
// 使用: from Yolo11DetTrt import YOLO11TRT
// ============================================================
PYBIND11_MODULE(Yolo11DetTrt, m) {
    m.doc() = "YOLO11 inference with TensorRT and CUDA pre/post processing";

    py::class_<YOLO11TRT>(m, "YOLO11TRT", py::module_local())
        .def(py::init<const std::string&, float, float, int>(),
             py::arg("engine_path"),
             py::arg("conf_thresh") = 0.5f,
             py::arg("iou_thresh") = 0.5f,
             py::arg("num_classes") = 80)
        .def("preprocess", &YOLO11TRT::preprocess, py::arg("image"),
             "CPU-compatible letterbox + normalize [0,1] + HWC->CHW")
        .def("infer_raw", &YOLO11TRT::infer_raw, py::arg("input_data"),
             "Raw TensorRT inference on preprocessed float tensor")
        .def("postprocess", &YOLO11TRT::postprocess, py::arg("raw_output"), py::arg("preproc_info"),
             "CPU-compatible decode detections -> NMS -> list of dicts")
        .def("infer", &YOLO11TRT::infer, py::arg("image"),
             "CPU-compatible end-to-end path (preprocess+infer_raw+postprocess)")
        .def("infer_cuda", &YOLO11TRT::infer_cuda, py::arg("image"),
             "CUDA pre/post optimized end-to-end path (alias of detect_cuda)")
        .def("detect_cuda", &YOLO11TRT::detect_cuda, py::arg("image"),
             "CUDA pre/post optimized end-to-end path (letterbox+infer+decode+NMS)")
        .def_property_readonly("input_name", &YOLO11TRT::get_input_name)
        .def_property_readonly("output_name", &YOLO11TRT::get_output_name)
        .def_property_readonly("input_shape", &YOLO11TRT::get_input_shape)
        .def_property_readonly("output_shape", &YOLO11TRT::get_output_shape);
}
