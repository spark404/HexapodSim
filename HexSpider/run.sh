#!/bin/bash

# export PATH=/usr/local/opt/ruby/bin:$PATH
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=`pwd`/models

gz sim world/hexspider_world.sdf 
