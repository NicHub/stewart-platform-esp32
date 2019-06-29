#!/usr/bin/env bash

# Compilation of `hexapod_desktop_app`
# Works on macOS Mojave

EXEC_NAME=hexapod_desktop_app
rm -f $EXEC_NAME

g++                               \
    -I ../src/                    \
    ../src/Hexapod_Kinematics.cpp \
    hexapod_desktop_app.cpp       \
    -o $EXEC_NAME                 \
    -std=c++11                    \
    && ./$EXEC_NAME
