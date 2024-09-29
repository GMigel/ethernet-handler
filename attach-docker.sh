#!/bin/bash

# docker attach eth-server
docker run -it --mount src=$PWD,target=/home/ethernet-server,type=bind -v “$(pwd):/home/ethernet-server” ethernet-server
