# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.13)

# The Bluetooth stacks (components/btstack, components/bluepad32) were deleted on
# 2026-08-08. Nothing referenced them — firmware.elf contained zero matching symbols,
# the legacy gamepad app (sketch.cpp) no longer exists, and they had been excluded
# from the build since 2026-07-31. Git history keeps them if the gamepad path is
# ever wanted again. The project() name below is a leftover from the template this
# repo was forked from.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(bluepad32_arduino_app_template)
