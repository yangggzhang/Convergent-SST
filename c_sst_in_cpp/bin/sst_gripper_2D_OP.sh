#!/bin/bash

for i in $(seq 1 30); do
    params="./run --config ../input/sst_gripper_2D_OP.cfg --trial $i"
    eval $params
done