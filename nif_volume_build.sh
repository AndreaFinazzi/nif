#!/usr/bin/env sh
docker build docker/clion/ -t usrg/clion

docker tag usrg/clion ghcr.io/andreafinazzi/clion
docker push ghcr.io/andreafinazzi/clion