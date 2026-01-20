cmake_minimum_required(VERSION 3.13)
project(mppi_py)

set(CMAKE_CXX_STANDARD 17)
# 开启优化选项，这对于 MPPI 这种采样算法至关重要
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -shared -fPIC -fopenmp")

# 寻找依赖库
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# 关键：包含路径配置
include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    src/shims                        # 1. 这里包含了 grid_map_core/GridMap.hpp
    src/mppi_controller/include      # 2. 这里包含 common.hpp, forward_mppi.hpp 等
)

# 定义源文件列表 (只包含核心算法，不包含 _ros.cpp)
set(MPPI_SOURCES
    src/mppi_controller/src/mpc_base.cpp
    src/mppi_controller/src/prior_samples_with_costs.cpp
    src/mppi_controller/src/stein_variational_guided_mppi.cpp
    # 如果你也想用 forward_mppi, 加上它:
    src/mppi_controller/src/forward_mppi.cpp 
    # 注意：绝对不要包含 mppi_controller_ros.cpp 或 node.cpp
)

# 生成 Python 模块 (.so 文件)
pybind11_add_module(mppi_py src/bind.cpp ${MPPI_SOURCES})
target_link_libraries(mppi_py PRIVATE OpenMP::OpenMP_CXX)