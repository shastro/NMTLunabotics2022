#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Add route from computer to AI cpu
sudo route add -net 169.254.0.0 gw 192.168.1.51 netmask 255.255.0.0 dev eth0

# this is set up in the bashrc and doesn't work anyway. --alex
# # Enable IP forwarding on control pi
# ssh pi@control \
#     sh -c \
#     'echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward'

# Add route from AI cpu to computer
ssh pi@control \
    ssh pi@ai \
    sudo route add -net 192.168.1.0 gw 169.254.10.11 netmask 255.255.255.0 dev eth0
