#!/bin/bash

export PATH=/usr/local/opt/ruby/bin:$PATH
export GZ_SIM_RESOURCE_PATH=/Users/hugo/Projects/HexapodSim/HexSpider/models

gz sim -s world/hexspider_world.sdf &
SERVER_PID=$!

gz sim -g

kill $SERVER_PID