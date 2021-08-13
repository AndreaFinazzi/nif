#!/usr/bin/env sh
docker build docker/ -t usrg/nifde:master --build-arg GH_TOKEN=$GH_TOKEN

docker tag usrg/nifde:master ghcr.io/andreafinazzi/ade:master
docker push ghcr.io/andreafinazzi/ade:master