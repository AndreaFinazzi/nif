#!/usr/bin/env sh
export GL_TOKEN=$(cat .secret.gitlab) 
export GH_TOKEN=$(cat .secret.github) 
docker build docker/volumes/galactic -t usrg/nif_ros2:galactic --build-arg GH_TOKEN=$GH_TOKEN --build-arg GL_TOKEN=$GL_TOKEN

docker tag  usrg/nif_ros2:galactic ghcr.io/andreafinazzi/nif_ros2:galactic
docker push ghcr.io/andreafinazzi/nif_ros2:galactic