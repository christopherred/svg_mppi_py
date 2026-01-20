#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// 引入原项目头文件
#include "mppi_controller/common.hpp"
#include "mppi_controller/stein_variational_guided_mppi.hpp"
// 引入我们的伪造库
#include <grid_map_core/GridMap.hpp>

namespace py = pybind11;
using namespace mppi;

PYBIND11_MODULE(mppi_py, m) {
    m.doc() = "SVG-MPPI Python Bindings";

    // 1. 绑定 GridMap (仅需要 Python 能调用的部分)
    py::class_<grid_map::GridMap>(m, "GridMap")
        .def(py::init<>())
        .def("setGeometry", [](grid_map::GridMap& gm, double lx, double ly, double res, double px, double py) {
            gm.setGeometry(grid_map::Length(lx, ly), res, grid_map::Position(px, py));
        })
        .def("add", &grid_map::GridMap::add); // Python端传 numpy array 即可

    // 2. 绑定 Params 结构体
    py::class_<Params> params(m, "Params");
    
    py::class_<Params::Common>(params, "Common")
        .def(py::init<>())
        .def_readwrite("thread_num", &Params::Common::thread_num)
        .def_readwrite("prediction_step_size", &Params::Common::prediction_step_size)
        .def_readwrite("prediction_interval", &Params::Common::prediction_interval)
        .def_readwrite("max_steer_angle", &Params::Common::max_steer_angle)
        .def_readwrite("reference_speed", &Params::Common::reference_speed)
        .def_readwrite("speed_prediction_mode", &Params::Common::speed_prediction_mode);
        // ... 根据需要在 config.yaml 中看到的参数继续添加

    py::class_<Params::SVGuidedMPPI>(params, "SVGuidedMPPI")
        .def(py::init<>())
        .def_readwrite("sample_batch_num", &Params::SVGuidedMPPI::sample_batch_num)
        .def_readwrite("lambda", &Params::SVGuidedMPPI::lambda)
        .def_readwrite("alpha", &Params::SVGuidedMPPI::alpha)
        .def_readwrite("steer_cov", &Params::SVGuidedMPPI::steer_cov)
        .def_readwrite("guide_sample_num", &Params::SVGuidedMPPI::guide_sample_num)
        .def_readwrite("svgd_step_size", &Params::SVGuidedMPPI::svgd_step_size)
        .def_readwrite("num_svgd_iteration", &Params::SVGuidedMPPI::num_svgd_iteration)
        .def_readwrite("is_use_nominal_solution", &Params::SVGuidedMPPI::is_use_nominal_solution)
        .def_readwrite("is_covariance_adaptation", &Params::SVGuidedMPPI::is_covariance_adaptation);

    // 3. 绑定 SVG-MPPI 控制器
    py::class_<cpu::SVGuidedMPPI>(m, "SVGuidedMPPI")
        .def(py::init<const Params::Common&, const Params::SVGuidedMPPI&>())
        .def("set_obstacle_map", &cpu::SVGuidedMPPI::set_obstacle_map)
        .def("set_reference_map", &cpu::SVGuidedMPPI::set_reference_map)
        .def("solve", &cpu::SVGuidedMPPI::solve)
        .def("get_predictive_seq", &cpu::SVGuidedMPPI::get_predictive_seq)
        .def("get_state_seq_candidates", &cpu::SVGuidedMPPI::get_state_seq_candidates)
        .def("get_cov_matrices", &cpu::SVGuidedMPPI::get_cov_matrices);
}