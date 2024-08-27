#!/bin/bash

#*******************************************************************************
# Last modified on 1 July 2023
# Author: Aditya Rauniyar, Krishna Suresh
# Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
# Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
# Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
#*******************************************************************************

# TODO: Add print statements and test this out with updated config file

find ../src/ -iname *.h -o -iname *.cpp | xargs clang-format -i -style=file
find ../include/ -iname *.h -o -iname *.cpp | xargs clang-format -i -style=file
