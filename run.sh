#!/bin/bash

export GZ_SIM_RESOURCE_PATH=/Users/hugo/Projects/Hexapod/Simulation/HexSpider/models


gz sim -s world/hexspider_world.sdf &
SERVER_PID=$!

gz sim -g

kill $SERVER_PID
