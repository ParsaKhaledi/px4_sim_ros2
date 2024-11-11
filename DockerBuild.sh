#!/bin/bash
DOCKERFILE=$1
echo ${DOCKERFILE}
docker build -t alienkh/px4_sim:latest -f {$dockerFileName} .