#!/bin/bash

xhost +
docker compose up &
./grid/create-grid &
python3 ./grid/visualizer.py