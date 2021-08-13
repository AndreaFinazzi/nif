#!/usr/bin/env sh
docker build docker/ -t usrg/nifde:ssh --build-arg GH_TOKEN=$CR_PAT

docker tag usrg/nifde:ssh ghcr.io/andreafinazzi/ade:ssh
docker push ghcr.io/andreafinazzi/ade:ssh