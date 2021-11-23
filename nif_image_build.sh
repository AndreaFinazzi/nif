#!/usr/bin/env sh
export GL_TOKEN=$(cat .secret.gitlab) 
export GH_TOKEN=$(cat .secret.github) 
docker build docker/ -t usrg/nifde:${ROS_DISTRO} --build-arg GH_TOKEN=$GH_TOKEN --build-arg GL_TOKEN=$GL_TOKEN

docker tag usrg/nifde:${ROS_DISTRO} ghcr.io/andreafinazzi/nif:${ROS_DISTRO}
docker push ghcr.io/andreafinazzi/nif:${ROS_DISTRO}