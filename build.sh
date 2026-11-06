#!/bin/bash
cd "$(dirname "$0")"
SVR=/opt/steamvr/bin/linuxarm64
gcc -O2 -Wall -Wno-missing-braces -Iinclude src/lensalign.c -o lensalign -L$SVR -lopenxr_loader -ljpeg -lEGL -lOpenGL -lm -Wl,-rpath,$SVR
