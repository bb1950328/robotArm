#!/usr/bin/env bash

g++ src/*.cpp -E | sed '/^#/ d' | sed -r '/^\s*$/d' > preprocessor_output.cpp