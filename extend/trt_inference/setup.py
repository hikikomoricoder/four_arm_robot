import os

import pybind11
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup
from setuptools.command.build_ext import build_ext

# ---------------------------------------------------------------------------
# Path configuration — adjust if your CUDA / TensorRT are located elsewhere
# ---------------------------------------------------------------------------
CUDA_HOME = os.environ.get("CUDA_HOME", "/usr/local/cuda")
NVCC = os.environ.get("NVCC", os.path.join(CUDA_HOME, "bin", "nvcc"))
TRT_INCDIR = "/usr/include/x86_64-linux-gnu"
TRT_LIBDIR = "/usr/lib/x86_64-linux-gnu"


def _nvcc_compile_args(cc_args):
    """Keep only include/define args that nvcc accepts directly."""
    result = []
    i = 0
    while i < len(cc_args):
        arg = cc_args[i]
        if arg.startswith("-I") or arg.startswith("-D") or arg.startswith("-U"):
            result.append(arg)
        elif arg == "-isystem" and i + 1 < len(cc_args):
            result.extend([arg, cc_args[i + 1]])
            i += 1
        i += 1
    return result


class BuildExtWithNvcc(build_ext):
    """Build normal C++ sources with g++ and the optimized TRT source with nvcc."""

    def build_extensions(self):
        original_compile = self.compiler._compile

        def compile_with_nvcc(obj, src, ext, cc_args, extra_postargs, pp_opts):
            if os.path.basename(src) in {"yolo11_detect_trt.cpp"}:
                cmd = [
                    NVCC,
                    "-x", "cu",
                    "-c", src,
                    "-o", obj,
                    "-std=c++17",
                    "-O3",
                    "--compiler-options", "-fPIC",
                    "--expt-relaxed-constexpr",
                ]
                cmd.extend(_nvcc_compile_args(cc_args + pp_opts))
                self.spawn(cmd)
            else:
                original_compile(obj, src, ext, cc_args, extra_postargs, pp_opts)

        self.compiler._compile = compile_with_nvcc
        try:
            super().build_extensions()
        finally:
            self.compiler._compile = original_compile


common_include_dirs = [
    pybind11.get_include(),
    f"{CUDA_HOME}/include",
    TRT_INCDIR,
]

common_library_dirs = [
    f"{CUDA_HOME}/lib64",
    TRT_LIBDIR,
]

common_libraries = [
    "nvinfer",
    "cudart",
]

ext_modules = [
    Pybind11Extension(
        "Yolo11DetTrt",
        ["yolo11_detect_trt.cpp"],
        include_dirs=common_include_dirs,
        library_dirs=common_library_dirs,
        libraries=common_libraries,
        cxx_std=17,
        extra_compile_args=["-Wno-deprecated-declarations"],
    ),
]

setup(
    name="trt_inference",
    version="0.2.0",
    description="nothing important",
    ext_modules=ext_modules,
    cmdclass={"build_ext": BuildExtWithNvcc},
    zip_safe=False,
)
