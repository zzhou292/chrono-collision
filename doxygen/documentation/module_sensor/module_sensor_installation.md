Install the SENSOR module   {#module_sensor_installation}
===============================

[TOC]

This is an optional module that enables the modeling and simulation of sensors for the purpose of simulating robotics and autonomous agents within Chrono.

Read [the introduction to modules](modularity.html) for a technical
background on the modularity of the Chrono project.


## Features

The **SENSOR module** allows users to model and simulate sensors for robotics and autonomous agents.

For more detail, read the [Chrono::Sensor](@ref manual_sensor) section of the reference manual.


## Required Dependencies

- To **run** applications based on this module, the following are required:
  * NVIDIA GPU, Maxwell or later - capable of running OptiX

- To **build** applications based on this module, the following are required:
  * [CUDA](https://developer.nvidia.com/cuda-downloads) - tested with version 10.0
  * [OptiX](https://developer.nvidia.com/designworks/optix/download) - version 6.5
  * [GLFW](https://www.glfw.org/) - version 3.0 or later
  * [GLEW](http://glew.sourceforge.net/) - version 1.0 or later
  * OpenGL
  * [TensoRT](https://developer.nvidia.com/tensorrt) (optional) - version 7.0.0

## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:

2. Set the `ENABLE_MODULE_SENSOR` as 'on', then press 'Configure' (to refresh the variable list)

3. Set the `OPTIX_INSTALL_DIR` variable to the root of the OptiX directory installed on the system (directory that includes `lib/` `bin/` `include/`) and press 'Configure' to refresh the variable list. If a refresh does not correctly set the following variables: `OptiX_Include`,`optix_LIBRARY`,`optix_prime_LIBRARY`, and `optixu_LIBRARY`, manually set them accordingly.

4. Optionally set `USE_CUDA_NVRTC` to 'on' to enable runtime compilation of the Optix RT Kernels. Press 'Configure' to refresh the variable list. If set to 'off', the RT Kernels will be compiled at runtime. Depending on the system, you may need to set `CUDA_ARCH_NAME` to the specific target architecture since this will result in RT Kernels being compiled to PTX.

5. Optionally set `USE_TENSOR_RT` to 'on' to enable use of TensorRT for augmenting sensor data. Press 'Configure' to refresh the variable list.
    * Set the `TENSOR_RT_INSTALL_DIR` variable to the root of the TensorRT directory installed on the system (directory that includes `lib/`, `bin/`, `include/`) and press 'Configure to refresh the avriable list'
    * If a refresh does not correctly set the following variables: `TENSOR_RT_INCLUDE_PATH`,`TENSOR_RT_NVINFER`,`TENSOR_RT_ONNXPARSER`, and `TENSOR_RT_PARSER`, manually set them accordingly with the last three variable pointing directly to their corresponding library files.

6. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Consult the [reference manual](@ref manual_sensor).

- Look at the [API section](@ref sensor) of this module for documentation about classes and functions.

- Look at the C++ and Python source of [demos](@ref tutorial_table_of_content_chrono_sensor) to learn how to use the functions of this module.
