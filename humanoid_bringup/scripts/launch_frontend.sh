#/bin/bash

set -e

docker container stop humanoid_frontend
docker container rm humanoid_frontend
docker pull chinaheyu/humanoid_frontend:latest

docker run --name humanoid_frontend --restart unless-stopped -p 80:80 -d chinaheyu/humanoid_frontend:latest
