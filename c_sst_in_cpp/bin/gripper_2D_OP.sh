#!/bin/bash

for i in $(seq 1 5); do
    params="./run --config ../input/sst_gripper_2D_OP.cfg --planner sst --trial $i"
    eval $params
done

for i in $(seq 1 5); do
    params="./run --config ../input/sst_gripper_2D_OP.cfg --planner rrt_restart --trial $i"
    eval $params
done

for i in $(seq 1 5); do
    params="./run --config ../input/sst_gripper_2D_OP.cfg --planner rrt --trial $i"
    eval $params
done
