#!/usr/bin/env bash

# Compilation of `hexapod_app`
# Works on macOS Mojave

g++                              \
    -I ../src/                   \
    ../src/Hexapod_Kinematics.cpp \
    hexapod_app.cpp              \
    -o hexapod_app               \
    -std=c++11                   \
    && ./hexapod_app
