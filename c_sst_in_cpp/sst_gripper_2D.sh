#!/bin/bash

for i in $(seq 1 30); do
    params="./bin/run --config ./input/sst_gripper_2D.cfg --trial $i"
    eval $params
done