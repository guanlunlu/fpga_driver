#!/bin/bash
USER=admin
HOST="169.254.105.68"

scp -r ~/corgi_ws/src/fpga_server/src \
    ~/corgi_ws/src/fpga_server/include \
    ~/corgi_ws/src/fpga_server/CMakeLists.txt \
    ~/corgi_ws/src/fpga_server/config \
    ~/corgi_ws/src/fpga_server/fpga_bitfile \
    ${USER}@${HOST}:~/corgi_ws/src/fpga_server

ssh ${USER}@${HOST} "cd ~/corgi_ws/build \
                    && make -j4"