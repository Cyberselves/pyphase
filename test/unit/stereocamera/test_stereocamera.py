#!/usr/bin/env python3

"""!
 @authors Ben Knight (bknight@i3drobotics.com)
 @date 2021-05-26
 @copyright Copyright (c) I3D Robotics Ltd, 2021
 @file test_stereocamera.py
 @brief Unit tests for Stereo Camera class
 @details Unit tests for use with PyTest
"""
import os
import numpy as np
from phase.pyphase.stereocamera import CameraDeviceInfo, createStereoCamera
from phase.pyphase.stereocamera import CameraDeviceType, CameraInterfaceType
from phase.pyphase import readImage


def test_StereoCamera():
    # Test initalisation of UVCStereoCamera using CameraDeviceInfo
    device_info = CameraDeviceInfo(
        "0", "0", "virtualuvc",
        CameraDeviceType.DEVICE_TYPE_GENERIC_UVC,
        CameraInterfaceType.INTERFACE_TYPE_VIRTUAL
    )
    createStereoCamera(device_info)


def test_StereoCamera():
    # Test initalisation of stereo camera using CameraDeviceInfo
    device_info = CameraDeviceInfo(
        "abc123left", "abc123right", "abc123unique",
        CameraDeviceType.DEVICE_TYPE_GENERIC_PYLON,
        CameraInterfaceType.INTERFACE_TYPE_USB
    )
    createStereoCamera(device_info)


def test_StereoCamera_isConnected_onInit():
    # Test if stereo camera is connected
    device_info = CameraDeviceInfo(
        "abc123left", "abc123right", "abc123unique",
        CameraDeviceType.DEVICE_TYPE_GENERIC_PYLON,
        CameraInterfaceType.INTERFACE_TYPE_USB
    )
    cam = createStereoCamera(device_info)
    assert cam.isConnected() is False


def test_StereoCamera_connect_onInit():
    # Test to connect stereo camera
    device_info = CameraDeviceInfo(
        "abc123left", "abc123right", "abc123unique",
        CameraDeviceType.DEVICE_TYPE_GENERIC_PYLON,
        CameraInterfaceType.INTERFACE_TYPE_USB
    )
    cam = createStereoCamera(device_info)
    assert cam.connect() is False
