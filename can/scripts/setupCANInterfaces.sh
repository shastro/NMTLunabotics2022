#!/bin/bash

sudo modprobe can
sudo ip link add dev can0 type can
sudo ip link set up can0
