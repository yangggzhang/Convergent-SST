#!/bin/bash

for i in $(seq 1 30); do
    params="./run --config ../input/sst_complex_hill.cfg --trial $i"
    eval $params
done

for i in $(seq 1 30); do
    params="./run --config ../input/sst_complex_hill.cfg --planner rrt --trial $i"
    eval $params
done

for i in $(seq 1 30); do
    params="./run --config ../input/sst_complex_hill.cfg --planner rrt_restart --trial $i"
    eval $params
done