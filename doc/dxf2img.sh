#!/bin/bash

# https://qcad.org/en/qcad-command-line-tools#dwg2bmp

# https://qcad.org/fr/qcad-command-line-tools#dwg2bmp

# Aliases to binaries.
shopt -s expand_aliases
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    alias dwg2bmp=$HOME"/opt/qcad-3.25.2-pro-linux-x86_64/dwg2bmp"
    alias dwg2svg=$HOME"/opt/qcad-3.25.2-pro-linux-x86_64/dwg2svg"
    alias inkscape='/snap/bin/inkscape'
elif [[ "$OSTYPE" == "darwin"* ]]; then
    alias dwg2bmp="/Applications/QCAD-Pro.app/Contents/Resources/dwg2bmp"
    alias dwg2svg="/Applications/QCAD-Pro.app/Contents/Resources/dwg2svg"
    alias inkscape='/Applications/Inkscape.app/Contents/MacOS/Inkscape'
else
    echo "$OSTYPE not supported"
    exit
fi

DWG2BMP()  {
    echo $BLOCK
    dwg2bmp                              \
        -antialiasing                    \
        -block="$BLOCK"                  \
        -background="white"              \
        -color-correction                \
        -color                           \
        -force                           \
        -quality=$QUALITY                \
        -resolution=$RESOLUTION          \
        -margin=0                        \
        -outfile=$FILE_OUT               \
        -layer="^(?!.*(0|geometry)).*$"  \
        $FILE_IN
    ls -la $FILE_OUT
}

DWG2SVG() {
    dwg2svg                              \
        -force                           \
        -outfile=$FILE_OUT               \
        -equal-margins=0                 \
        -block="$BLOCK"                  \
        -layer=$LAYER                    \
        $FILE_IN
}

OPTIMIZE-PNG()  {
    pngquant --strip --speed 1 --verbose --force --skip-if-larger 20 $FILE_OUT --output $FILE_OUT
}

ARA-FLUID-DIAGRAM-WO-FILTER() {
    BLOCK="*Paper_Space"
    FILE_IN=../ara-fluid-diagram.dxf
    FILE_OUT=../../ARA-FLUID-DIAGRAM-WO-FILTERS.svg
    LAYER="^(?!.*(0|geometry|w_filters|legend_w_filters)).*$"
    DWG2SVG
}

ARA-FLUID-DIAGRAM-W-FILTER() {
    BLOCK="*Paper_Space"
    FILE_IN=../ara-fluid-diagram.dxf
    FILE_OUT=../../ARA-FLUID-DIAGRAM_new.svg
    LAYER="^(?!.*(0|geometry|wo_filters|legend_wo_filters)).*$"
    DWG2SVG
}

ARA-SYSTEM-OVERVIEW-ELECTRICAL() {
    BLOCK="*Paper_Space"
    FILE_IN=../ARA-SYSTEM-OVERVIEW.dxf
    FILE_OUT=../../ARA-SYSTEM-OVERVIEW-ELECTRICAL.svg
    LAYER="^(?!.*(0|geometry)).*$"
    DWG2SVG
}

ARA-OVERALL-DIMENSIONS-MODULES-UP-BACK-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-OVERALL-DIMENSIONS-MODULES-UP-BACK-VIEW.png
    BLOCK="*Paper_Space0"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-OVERALL-DIMENSIONS-MODULES-UP-TOP-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-OVERALL-DIMENSIONS-MODULES-UP-TOP-VIEW.png
    BLOCK="*Paper_Space1"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-OVERALL-DIMENSIONS-MODULES-DOWN-BACK-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-OVERALL-DIMENSIONS-MODULES-DOWN-BACK-VIEW.png
    BLOCK="*Paper_Space2"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-OVERALL-DIMENSIONS-MODULES-DOWN-TOP-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-OVERALL-DIMENSIONS-MODULES-DOWN-TOP-VIEW.png
    BLOCK="*Paper_Space3"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-DESCRIPTION-MODULES-DOWN-BACK-AND-LEFT-PERSP-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-DESCRIPTION-MODULES-DOWN-BACK-AND-LEFT-PERSP-VIEW.png
    BLOCK="*Paper_Space4"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-DESCRIPTION-MODULES-DOWN-FRONT-AND-RIGHT-PERSP-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-DESCRIPTION-MODULES-DOWN-FRONT-AND-RIGHT-PERSP-VIEW.png
    BLOCK="*Paper_Space5"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-DESCRIPTION-MODULES-DOWN-FRONT-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-DESCRIPTION-MODULES-DOWN-FRONT-VIEW.png
    BLOCK="*Paper_Space6"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-DESCRIPTION-MODULES-DOWN-BOTTOM-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-DESCRIPTION-MODULES-DOWN-BOTTOM-VIEW.png
    BLOCK="*Paper_Space7"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

TANKS-DESCRIPTION-RIGHT-AND-BACK-PERSP-VIEW() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../TANKS-DESCRIPTION-RIGHT-AND-BACK-PERSP-VIEW.png
    BLOCK="*Paper_Space8"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-ROAD-SAFETY-MODULES-UP() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-ROAD-SAFETY-MODULES-UP.png
    BLOCK="*Paper_Space9"
    QUALITY=100
    RESOLUTION=5
    DWG2BMP
    OPTIMIZE-PNG
}

ARA-OVERALL-DIMENSIONS() {
    FILE_IN=../machine-specifications-01.dxf
    FILE_OUT=../../ARA-OVERALL-DIMENSIONS.png
    BLOCK="*Paper_Space10"
    QUALITY=100
    RESOLUTION=2
    DWG2BMP
    OPTIMIZE-PNG
}


###
#
# USAGE
#
# - All images referenced in the DXF must be in the same directory.
#   If not, QCAD won’t find them if the directory structure changes.
#   In case images cannot be found it is possible to edit the DXF in
#   a text editor, search “.jpg” or “.png” and modify the paths manualy.
#
# Uncomment desired output in the code below.
# Run the script.
#
##

# ARA-OVERALL-DIMENSIONS-MODULES-UP-BACK-VIEW
# ARA-OVERALL-DIMENSIONS-MODULES-UP-TOP-VIEW
# ARA-OVERALL-DIMENSIONS-MODULES-DOWN-BACK-VIEW
# ARA-OVERALL-DIMENSIONS-MODULES-DOWN-TOP-VIEW

# ARA-DESCRIPTION-MODULES-DOWN-BACK-AND-LEFT-PERSP-VIEW
# ARA-DESCRIPTION-MODULES-DOWN-FRONT-AND-RIGHT-PERSP-VIEW
# ARA-DESCRIPTION-MODULES-DOWN-FRONT-VIEW
# ARA-DESCRIPTION-MODULES-DOWN-BOTTOM-VIEW
TANKS-DESCRIPTION-RIGHT-AND-BACK-PERSP-VIEW
# ARA-ROAD-SAFETY-MODULES-UP
# ARA-OVERALL-DIMENSIONS
# ARA-SYSTEM-OVERVIEW-ELECTRICAL

# ARA-FLUID-DIAGRAM-WO-FILTER
# ARA-FLUID-DIAGRAM-W-FILTER
