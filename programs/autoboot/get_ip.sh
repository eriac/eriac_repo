#!/bin/sh
IFACE="eth0"
IP_ADDRESS=$(/sbin/ip -f inet -o addr show "${IFACE}" | cut -d\  -f 7 | cut -d/ -f 1)
echo $IP_ADDRESS
