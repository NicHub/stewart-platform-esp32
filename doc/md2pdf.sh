#!/usr/bin/env bash

pandoc                               \
    --from markdown                  \
    --to html                        \
    --standalone                     \
    --mathml                         \
    hexapod-kinematics.md            \
    -o hexapod-kinematics.html

prince --javascript hexapod-kinematics.html

