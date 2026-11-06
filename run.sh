#!/bin/bash
# Starts the lens alignment app (log: ~/lensalign/lensalign.log). Quit from the SteamVR dashboard or Ctrl+C.
cd "$(dirname "$0")"
[ -x lensalign ] || ./build.sh || exit 1
exec ./lensalign "$@" 2>&1 | tee lensalign.log
