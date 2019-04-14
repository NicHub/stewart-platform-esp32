#!/usr/bin/env bash

# Compilation of `hexapod_app`
# Works on macOS Mojave

g++                              \
    -I ../src/                   \
    ../src/HexapodKinematics.cpp \
    hexapod_app.cpp              \
    -o hexapod_app               \
    -std=c++11                   \
    && ./hexapod_app             \
    && cat angles.txt
