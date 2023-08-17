#!/bin/env bash

set -euxo pipefail
echo "$USER    -   rtprio   98" | sudo tee -a /etc/security/limits.conf
