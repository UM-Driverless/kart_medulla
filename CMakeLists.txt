# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.13)

# Do not build the Bluetooth stacks. They are compiled and then discarded: nothing
# references them, so the linker drops every symbol. Measured 2026-07-31 before this
# line existed — libbtstack.a 7.6 MB and libbluepad32.a 2.0 MB were produced, and
# firmware.elf contained ZERO symbols matching uni_hid|btstack_|bluepad out of 5330.
# They were 189 of the 1174 objects in a cold build, bought nothing, and cost time on
# every clean build.
#
# Nothing reaches them: main/main.c never mentions either, main/CMakeLists.txt has
# sketch.cpp (the legacy Bluepad32 gamepad app) commented out of srcs, and that file
# no longer exists in the repo. The only other mention is this project() name below,
# which is a leftover from the template this repo was forked from.
#
# TO RE-ENABLE (if the gamepad path is ever wanted again): delete this line. The
# components are still present under components/, untouched.
set(EXCLUDE_COMPONENTS btstack bluepad32)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(bluepad32_arduino_app_template)
