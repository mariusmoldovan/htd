#!/bin/bash

STORAGE_DIRECTORY=`dirname "$(readlink -f "$0")"`

exec "${STORAGE_DIRECTORY}/htd_main" --input gr --output td --ordering min-fill --opt width --iterations 0 --print-opt-progress "$@" <&0
