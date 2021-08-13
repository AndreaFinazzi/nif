#!/usr/bin/env sh
docker build docker/ -t usrg/nifde:master

docker tag usrg/nifde:master ghcr.io/andreafinazzi/ade:master
docker push ghcr.io/andreafinazzi/ade:master