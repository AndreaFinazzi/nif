#!/usr/bin/env sh
docker build docker/ -t usrg/nifde:ssh

docker tag usrg/nifde:ssh ghcr.io/andreafinazzi/ade:ssh
docker push ghcr.io/andreafinazzi/ade:ssh