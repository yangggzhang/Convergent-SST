#!/bin/bash

for i in $(seq 60); do
    params="./run --planner rrt --config ../input/hill.cfg --trial $i"
    eval $params
done
