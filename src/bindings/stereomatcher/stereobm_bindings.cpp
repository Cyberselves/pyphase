/*!
 * @authors Ben Knight (bknight@i3drobotics.com)
 * @date 2021-05-26
 * @copyright Copyright (c) I3D Robotics Ltd, 2021
 * 
 * @file stereobm_bindings.cpp
 * @brief Stereo Block Matcher class python bindings
 * @details Python bindings generated using pybind11
 */

#include "pybind11/pybind11.h"
#include "pybind11/functional.h"
#include "ndarray_converter.h"

#include <phase/stereomatcher/stereobm.h>

namespace py = pybind11;

void init_stereobm(py::module_ &m) {
    NDArrayConverter::init_numpy();
    py::class_<I3DR::Phase::StereoBM>(m, "StereoBM", R"(
        Class to set the parameters for stereoBM matcher
        )")
        .def(py::init<>())
        .def(py::init<I3DR::Phase::StereoParams>(), R"(
            Stereo parameters contain matcherType, windowSize, minDisparity, numDisparities, interpolation
            
            )")
        .def("compute", &I3DR::Phase::StereoBM::compute, R"(
            Compute stereo matching

            Parameters
            ----------
            left_image : numpy.ndarray
                Left image of stereo pair
            right_image : numpy.ndarray
                Right image of stereo pair
            )")
        .def("startComputeThread", &I3DR::Phase::StereoBM::startComputeThread, R"(
            Start compute thread

            Parameters
            ----------
            left_image : numpy.ndarray
                Left image of stereo pair
            right_image : numpy.ndarray
                Right image of stereo pair
            )")
        .def("setComputeThreadCallback", &I3DR::Phase::StereoBM::setComputeThreadCallback, R"(
            Set to compute thread callback

            Parameters
            ----------
            computeThread_callback : std::function<void __cdecl(void)>
            )")
        .def("isComputeThreadRunning", &I3DR::Phase::StereoBM::isComputeThreadRunning, R"(
            Check if compute thread is running

            Returns
            -------
            bool
                True is compute thread is running
            )")
        .def("getComputeThreadResult", &I3DR::Phase::StereoBM::getComputeThreadResult, R"(
            Get compute thread result 

            )")
        .def("setWindowSize", &I3DR::Phase::StereoBM::setWindowSize, R"(
            Set window size value

            Parameters
            ----------
            value : int
                Desired value of window size value
            )")
        .def("setMinDisparity", &I3DR::Phase::StereoBM::setMinDisparity, R"(
            Set minimum disparity value
            Parameters
            ----------
            value : int
                Desired value of minimum disparity value
        )")
        .def("setNumDisparities", &I3DR::Phase::StereoBM::setNumDisparities, R"(
            Set number of disparities

            Parameters
            ----------
            value : int
                Desired value of number of disparities
            )");
}