#!/bin/bash

gnome-terminal -t "编译" -x bash -c "g++ -o graph_slam ./*.cpp ./matplotlibcpp.h -I /usr/include/python2.7 -l python2.7;exec bash;"





