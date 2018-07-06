#!/bin/bash

for i in $(seq 3); do
    params="./run --planner rrt --trial $i"
    eval $params
done
