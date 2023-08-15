#/bin/bash

set -e

docker run --name humanoid_frontend --restart unless-stopped -p 80:80 -d chinaheyu/humanoid_frontend:latest
