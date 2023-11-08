#/usr/bin/bash

set -e

sudo rmmod cdc_acm
sudo modprobe cdc_acm
