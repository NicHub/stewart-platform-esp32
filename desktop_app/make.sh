#!/usr/bin/env bash

g++ -I ../src/ ../src/HexapodKinematics.cpp test.cpp -o app -std=c++11 && ./app
